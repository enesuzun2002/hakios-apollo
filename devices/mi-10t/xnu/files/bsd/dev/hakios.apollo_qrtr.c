/*
 * apollo: QRTR over the QCA6390's MHI IPCR channels (20 to the chip, 21 from it), as the kernel's QMI transport
 * (sys/hakios.qrtr.h, installed with hakios_qmi_set_transport(); the QMI helpers in hakios.qmi.c are shared).
 *
 * The router logic is the one of redwood's GLINK transport (bsd/dev/hakios.wifi.c, "qrtr_*", lines ~2726-3050), copied
 * here and adapted -- redwood's file is not touched. What differs, from Linux (net/qrtr/mhi.c, net/qrtr/af_qrtr.c,
 * net/qrtr/ns.c; all fetched from git.kernel.org torvalds/linux):
 *   - MHI: one MHI transfer is one QRTR packet (mhi.c: qcom_mhi_qrtr_send queues the skb as is, MHI_EOT; the download
 *     callback hands each buffer whole to qrtr_endpoint_post). apollo_ipcr_send()/set_receive() are exactly that.
 *   - Packet: version 1 header (af_qrtr.c qrtr_node_enqueue: 32-bit version, type, src node, src port, confirm_rx, size,
 *     dst node, dst port, little endian) and the payload padded to 4 bytes (skb_put_padto(ALIGN(len, 4) + hdr)); a
 *     receiver requires len == ALIGN(size, 4) + header (qrtr_endpoint_post), version 1 or 3 (v2).
 *   - Nodes: the local node id is 1 (qrtr_local_nid). The chip's is not known until its first packet (Linux:
 *     QRTR_EP_NID_AUTO, node->nid assigned from the first src node in qrtr_endpoint_post); control packets to a node not
 *     yet known carry dst node 0xffffffff, as node->nid does in Linux until then.
 *   - HELLO: Linux's endpoint registration sends HELLO at once (qrtr_endpoint_register -> qrtr_hello_work; from
 *     {node 1, CTRL port} to {node AUTO, CTRL port}) and nothing else is sent before it (qrtr_node_enqueue: -EAGAIN until
 *     hello_sent). Here apollo_qrtr_init() does the same. The chip's HELLO is then only logged: Linux's name service
 *     answers a HELLO by announcing its own servers (ns.c ctrl_cmd_hello -> announce_servers), and there are none here, so
 *     no second HELLO is sent (redwood's answer-once rule is not needed: ours already went out first).
 *   - BYE / DEL_SERVER / DEL_CLIENT: as ns.c: BYE removes every server of that node, DEL_SERVER one, a local port's
 *     close() sends DEL_CLIENT (ns.c ctrl_cmd_del_client) to the chip's node.
 *   - No flow control: Linux sets confirm_rx on data after a high watermark of 10 unconfirmed messages per port (qrtr_tx_wait).
 *     We never set it, so the chip never answers RESUME_TX; an incoming confirm_rx is answered, an incoming RESUME_TX ignored.
 *   - Threading: the receive function of apollo_ipcr_* runs on one kernel thread; lookup()'s catch-up of servers that were
 *     already there is done on a thread of our own (there is no link loop here to do it), both under qrtr_calling so
 *     callbacks stay one at a time.
 */
#include <kern/kalloc.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.apollo_wifi.h>
#include <sys/hakios.qrtr.h>
#include <sys/systm.h>

extern void thread_set_thread_name(thread_t thread, const char *name);    /* (shown by the hitch tracer, apollo_hitch=1) */

extern void loader_log_putc(char c);

static int say(const char *format, ...) __printflike(1, 2);

static int
say(const char *format, ...)
{
	char line[300];
	va_list list;

	va_start(list, format);
	vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
	return printf("%s", line);
}

#define HAKIOS_QRTR_MAX        8192            /* the most a datagram holds, either way (sys/hakios.qrtr.h) */
#define QRTR_US                 1u              /* qrtr_local_nid, af_qrtr.c */
#define QRTR_CONTROL            0xfffffffeu     /* QRTR_PORT_CTRL */
#define QRTR_NODE_UNKNOWN       0xffffffffu     /* QRTR_EP_NID_AUTO as it goes on the wire */
#define QRTR_FIRST_PORT         0x4000u         /* where Linux begins numbering ports that ask for no number */
enum { QRTR_DATA = 1, QRTR_HELLO = 2, QRTR_BYE = 3, QRTR_NEW_SERVER = 4, QRTR_DEL_SERVER = 5, QRTR_DEL_CLIENT = 6, QRTR_RESUME_TX = 7 };

struct qrtr_header {                            /* version 1 (struct qrtr_hdr_v1, af_qrtr.c) */
	uint32_t version, type, from_node, from_port, confirm, size, to_node, to_port;
};
struct qrtr_control {                           /* struct qrtr_ctrl_pkt: 20 bytes */
	uint32_t command;
	uint32_t words[4];                      /* a server: service, instance, node, port; a client: node, port */
};

struct hakios_qrtr_port {
	uint32_t number;                        /* 0: free */
	hakios_qrtr_receive_t receive;
	hakios_qrtr_server_t server;
	void *context;
	struct {
		uint32_t service, instance;     /* the instance as on the wire: version | instance << 8; 0: any */
		bool asked;
		bool fresh;                     /* not yet told of the services that were there before it */
	} lookups[4];
};

static LCK_GRP_DECLARE(apollo_qrtr_grp, "apollo-qrtr");
static LCK_MTX_DECLARE(qrtr_lock, &apollo_qrtr_grp);            /* the tables below */
static LCK_MTX_DECLARE(qrtr_calling, &apollo_qrtr_grp);         /* held around every callback into the QMI helpers */
static struct hakios_qrtr_port qrtr_ports[16];
static struct {
	uint32_t service, instance, node, port;
	bool there;
} qrtr_servers[64];
static uint32_t qrtr_peer;                      /* the chip's node; 0: it has not said */
static uint32_t qrtr_next_port = QRTR_FIRST_PORT, qrtr_in, qrtr_out;
static thread_t calling_thread;                 /* the thread that holds qrtr_calling */
static bool qrtr_started, qrtr_fresh_pending;
static int qrtr_debug;                          /* boot-arg apollo_qrtr_debug=1: data packets too */

static uint32_t
qrtr_peer_node(void)
{
	return qrtr_peer != 0 ? qrtr_peer : QRTR_NODE_UNKNOWN;
}

/* One datagram out, of any type. */
static int
qrtr_out_packet(uint32_t type, uint32_t from_port, uint32_t to_node, uint32_t to_port, const void *data, uint32_t length)
{
	uint32_t padded = (length + 3) & ~3u;
	struct qrtr_header *packet;
	int error;

	if (length == 0 || length > HAKIOS_QRTR_MAX) {
		return EMSGSIZE;
	}
	packet = kalloc_data(sizeof(*packet) + padded, Z_WAITOK | Z_ZERO);
	if (packet == NULL) {
		return ENOMEM;
	}
	packet->version = 1;
	packet->type = type;
	packet->from_node = QRTR_US;
	packet->from_port = from_port;
	packet->size = length;
	packet->to_node = to_port == QRTR_CONTROL && to_node == 0 ? qrtr_peer_node() : to_node;
	packet->to_port = to_port;
	memcpy(packet + 1, data, length);
	error = apollo_ipcr_send(packet, (uint32_t)sizeof(*packet) + padded);
	kfree_data(packet, sizeof(*packet) + padded);
	if (error == 0) {
		qrtr_out++;
	} else {
		say("apollo qrtr: send of type %u to node %u port %#x failed: %d\n", type, to_node, to_port, error);
	}
	if (qrtr_debug && (type == QRTR_DATA || error == 0)) {
		say("apollo qrtr: out type %u, %u bytes, port %#x -> node %u port %#x\n", type, length, from_port, to_node, to_port);
	}
	return error;
}

static bool
qrtr_matches(uint32_t service, uint32_t instance, unsigned server)
{
	return qrtr_servers[server].there && qrtr_servers[server].service == service &&
	       (instance == 0 || qrtr_servers[server].instance == instance);
}

/* Everybody who looked a service up is told that it has come, or gone. Called with qrtr_calling held. */
static void
qrtr_tell(uint32_t service, uint32_t instance, uint32_t node, uint32_t port, bool arrived)
{
	for (unsigned p = 0; p < sizeof(qrtr_ports) / sizeof(qrtr_ports[0]); p++) {
		for (unsigned l = 0; l < 4; l++) {
			hakios_qrtr_server_t server = NULL;
			void *context;

			lck_mtx_lock(&qrtr_lock);
			if (qrtr_ports[p].number != 0 && qrtr_ports[p].lookups[l].asked && !qrtr_ports[p].lookups[l].fresh &&
			    qrtr_ports[p].lookups[l].service == service &&
			    (qrtr_ports[p].lookups[l].instance == 0 || qrtr_ports[p].lookups[l].instance == instance)) {
				server = qrtr_ports[p].server;
			}
			context = qrtr_ports[p].context;
			lck_mtx_unlock(&qrtr_lock);
			if (server != NULL) {
				server(context, arrived, service, instance & 0xff, instance >> 8, node, port);
			}
		}
	}
}

/* The control port's part: Linux's name service (ns.c), without services of our own to tell of. */
static void
qrtr_control(const struct qrtr_header *header, const struct qrtr_control *control)
{
	int found = -1;

	switch (control->command) {
	case QRTR_HELLO:
		say("apollo qrtr: HELLO from node %u (ours went out at init)\n", header->from_node);
		break;
	case QRTR_NEW_SERVER:
		lck_mtx_lock(&qrtr_lock);
		for (unsigned i = 0; i < sizeof(qrtr_servers) / sizeof(qrtr_servers[0]); i++) {
			if (qrtr_servers[i].there && qrtr_servers[i].node == control->words[2] && qrtr_servers[i].port == control->words[3]) {
				found = (int)i;
				break;
			}
			if (!qrtr_servers[i].there && found < 0) {
				found = (int)i;
			}
		}
		if (found >= 0) {
			qrtr_servers[found].there = true;
			qrtr_servers[found].service = control->words[0];
			qrtr_servers[found].instance = control->words[1];
			qrtr_servers[found].node = control->words[2];
			qrtr_servers[found].port = control->words[3];
		}
		lck_mtx_unlock(&qrtr_lock);
		say("apollo qrtr: NEW_SERVER service %u (%#x), version %u, instance %u, node %u, port %u%s\n", control->words[0],
		    control->words[0], control->words[1] & 0xff, control->words[1] >> 8, control->words[2], control->words[3],
		    found < 0 ? " (table full: not kept)" : "");
		qrtr_tell(control->words[0], control->words[1], control->words[2], control->words[3], true);
		break;
	case QRTR_DEL_SERVER:
	case QRTR_BYE:
		for (unsigned i = 0; i < sizeof(qrtr_servers) / sizeof(qrtr_servers[0]); i++) {
			bool gone;

			lck_mtx_lock(&qrtr_lock);
			gone = qrtr_servers[i].there && (control->command == QRTR_BYE ? qrtr_servers[i].node == header->from_node :
			    qrtr_servers[i].node == control->words[2] && qrtr_servers[i].port == control->words[3]);
			qrtr_servers[i].there = qrtr_servers[i].there && !gone;
			lck_mtx_unlock(&qrtr_lock);
			if (gone) {
				say("apollo qrtr: %s: service %u at node %u port %u has gone\n", control->command == QRTR_BYE ? "BYE" : "DEL_SERVER",
				    qrtr_servers[i].service, qrtr_servers[i].node, qrtr_servers[i].port);
				qrtr_tell(qrtr_servers[i].service, qrtr_servers[i].instance, qrtr_servers[i].node, qrtr_servers[i].port, false);
			}
		}
		break;
	default:
		break;                                  /* (DEL_CLIENT, EXIT, PING, lookups: nothing here wants them) */
	}
}

/* One datagram from the channel (one MHI transfer). From the receive thread, with qrtr_calling held. */
static void
qrtr_input(const uint8_t *data, uint32_t length)
{
	struct qrtr_header header;
	hakios_qrtr_receive_t receive = NULL;
	void *context = NULL;
	uint32_t at;

	if (length >= sizeof(header) && data[0] == 1) {
		memcpy(&header, data, sizeof(header));
		at = sizeof(header);
	} else if (length >= 16 && data[0] == 3) {
		/* version 2: version, type, flags, a length of options; the size; then four numbers of 16 bits */
		bzero(&header, sizeof(header));
		header.type = data[1];
		header.confirm = data[2] & 1;
		memcpy(&header.size, data + 4, 4);
		header.from_node = (uint32_t)(data[8] | data[9] << 8);
		header.from_port = (uint32_t)(data[10] | data[11] << 8);
		header.to_node = (uint32_t)(data[12] | data[13] << 8);
		header.to_port = (uint32_t)(data[14] | data[15] << 8);
		header.from_port = header.from_port == 0xfffe ? QRTR_CONTROL : header.from_port;
		header.to_port = header.to_port == 0xfffe ? QRTR_CONTROL : header.to_port;
		at = 16 + data[3];
	} else {
		say("apollo qrtr: %u bytes that are no packet (they begin %#x)\n", length, length ? data[0] : 0);
		return;
	}
	if (header.size == 0 || header.size > length || at + header.size > length) {
		say("apollo qrtr: a packet of %u bytes in a transfer of %u: dropped\n", header.size, length);
		return;
	}
	qrtr_in++;
	if (qrtr_peer == 0 && header.from_node != 0 && header.from_node != QRTR_NODE_UNKNOWN) {
		qrtr_peer = header.from_node;           /* (qrtr_node_assign of the first source node) */
		say("apollo qrtr: the chip is node %u\n", qrtr_peer);
	}
	if (qrtr_debug) {
		say("apollo qrtr: in type %u, %u bytes, node %u port %#x -> port %#x\n", header.type, header.size, header.from_node,
		    header.from_port, header.to_port);
	}
	if (header.type == QRTR_RESUME_TX) {
		return;                                 /* (we never set confirm_rx, so none is asked for) */
	}
	if (header.confirm) {
		/* the sender counts what it has sent to this port, and waits for this before it sends many more */
		struct qrtr_control resume = { .command = QRTR_RESUME_TX, .words = { header.to_node, header.to_port } };

		qrtr_out_packet(QRTR_RESUME_TX, header.to_port, header.from_node, header.from_port, &resume, sizeof(resume));
	}
	if (header.to_port == QRTR_CONTROL || header.to_port == 0) {    /* (0: QRTR_PORT_CTRL_LEGACY) */
		struct qrtr_control control = { 0 };

		if (header.type == QRTR_DATA) {
			return;
		}
		memcpy(&control, data + at, header.size < sizeof(control) ? header.size : sizeof(control));
		qrtr_control(&header, &control);
		return;
	}
	lck_mtx_lock(&qrtr_lock);
	for (unsigned p = 0; p < sizeof(qrtr_ports) / sizeof(qrtr_ports[0]); p++) {
		if (qrtr_ports[p].number == header.to_port && header.type == QRTR_DATA) {
			receive = qrtr_ports[p].receive;
			context = qrtr_ports[p].context;
		}
	}
	lck_mtx_unlock(&qrtr_lock);
	if (receive != NULL) {
		receive(context, header.from_node, header.from_port, data + at, header.size);
	} else if (header.type == QRTR_DATA) {
		say("apollo qrtr: a packet for port %#x, which is nobody's: dropped\n", header.to_port);
	}
}

/* The receive function of the IPCR channel: a kernel thread, one packet at a time. */
static void
qrtr_receive(const void *data, unsigned long length, __unused void *context)
{
	lck_mtx_lock(&qrtr_calling);
	calling_thread = current_thread();
	qrtr_input(data, (uint32_t)length);
	calling_thread = THREAD_NULL;
	lck_mtx_unlock(&qrtr_calling);
}

/* Lookups made since the last look are told of the services that were there before them. With qrtr_calling held. */
static void
qrtr_fresh(void)
{
	for (unsigned p = 0; p < sizeof(qrtr_ports) / sizeof(qrtr_ports[0]); p++) {
		for (unsigned l = 0; l < 4; l++) {
			hakios_qrtr_server_t server;
			uint32_t service, instance;
			void *context;

			lck_mtx_lock(&qrtr_lock);
			if (qrtr_ports[p].number == 0 || !qrtr_ports[p].lookups[l].fresh) {
				lck_mtx_unlock(&qrtr_lock);
				continue;
			}
			qrtr_ports[p].lookups[l].fresh = false;
			server = qrtr_ports[p].server;
			service = qrtr_ports[p].lookups[l].service;
			instance = qrtr_ports[p].lookups[l].instance;
			context = qrtr_ports[p].context;
			lck_mtx_unlock(&qrtr_lock);
			for (unsigned i = 0; i < sizeof(qrtr_servers) / sizeof(qrtr_servers[0]) && server != NULL; i++) {
				if (qrtr_matches(service, instance, i)) {
					server(context, true, qrtr_servers[i].service, qrtr_servers[i].instance & 0xff,
					    qrtr_servers[i].instance >> 8, qrtr_servers[i].node, qrtr_servers[i].port);
				}
			}
		}
	}
}

/* Ours: waits for lookup() to have asked, then does what qrtr_fresh() says, apart from the receive thread. */
static void
qrtr_fresh_thread(__unused void *parameter, __unused wait_result_t result)
{
	for (;;) {
		lck_mtx_lock(&qrtr_lock);
		while (!qrtr_fresh_pending) {
			lck_mtx_sleep(&qrtr_lock, LCK_SLEEP_DEFAULT, (event_t)&qrtr_fresh_pending, THREAD_UNINT);
		}
		qrtr_fresh_pending = false;
		lck_mtx_unlock(&qrtr_lock);
		lck_mtx_lock(&qrtr_calling);
		calling_thread = current_thread();
		qrtr_fresh();
		calling_thread = THREAD_NULL;
		lck_mtx_unlock(&qrtr_calling);
	}
}

static struct hakios_qrtr_port *
qrtr_open(hakios_qrtr_receive_t receive, hakios_qrtr_server_t server, void *context)
{
	struct hakios_qrtr_port *port = NULL;

	lck_mtx_lock(&qrtr_lock);
	for (unsigned p = 0; p < sizeof(qrtr_ports) / sizeof(qrtr_ports[0]) && port == NULL; p++) {
		if (qrtr_ports[p].number == 0) {
			port = &qrtr_ports[p];
			bzero(port, sizeof(*port));
			port->number = qrtr_next_port++;
			port->receive = receive;
			port->server = server;
			port->context = context;
		}
	}
	lck_mtx_unlock(&qrtr_lock);
	return port;
}

static int
qrtr_send(struct hakios_qrtr_port *port, uint32_t node, uint32_t to, const void *data, size_t length)
{
	if (port == NULL || port->number == 0 || length > HAKIOS_QRTR_MAX) {
		return EINVAL;
	}
	return qrtr_out_packet(QRTR_DATA, port->number, node, to, data, (uint32_t)length);
}

static int
qrtr_lookup(struct hakios_qrtr_port *port, uint32_t service, uint32_t version, uint32_t instance)
{
	int error = ENOSPC;

	lck_mtx_lock(&qrtr_lock);
	for (unsigned l = 0; l < 4 && error != 0 && port != NULL && port->number != 0; l++) {
		if (!port->lookups[l].asked) {
			port->lookups[l].service = service;
			port->lookups[l].instance = instance != 0 ? (version & 0xff) | instance << 8 : 0;
			port->lookups[l].asked = true;
			port->lookups[l].fresh = true;
			error = 0;
		}
	}
	if (error == 0) {
		qrtr_fresh_pending = true;
		thread_wakeup((event_t)&qrtr_fresh_pending);
	}
	lck_mtx_unlock(&qrtr_lock);
	return error;
}

static void
qrtr_close(struct hakios_qrtr_port *port)
{
	struct qrtr_control gone = { .command = QRTR_DEL_CLIENT, .words = { QRTR_US, port ? port->number : 0 } };
	bool outside = current_thread() != calling_thread;

	if (port == NULL || port->number == 0) {
		return;
	}
	if (outside) {
		lck_mtx_lock(&qrtr_calling);            /* (a callback of the port's that is running ends first) */
	}
	lck_mtx_lock(&qrtr_lock);
	port->number = 0;
	lck_mtx_unlock(&qrtr_lock);
	if (outside) {
		lck_mtx_unlock(&qrtr_calling);
	}
	if (qrtr_peer != 0) {
		qrtr_out_packet(QRTR_DEL_CLIENT, gone.words[1], qrtr_peer, QRTR_CONTROL, &gone, sizeof(gone));
	}
}

static const struct hakios_qrtr_ops qrtr_ops = { .open = qrtr_open, .send = qrtr_send, .lookup = qrtr_lookup, .close = qrtr_close };

int
apollo_qrtr_init(void)
{
	struct qrtr_control hello = { .command = QRTR_HELLO };
	thread_t thread;
	int error;

	lck_mtx_lock(&qrtr_lock);
	if (qrtr_started) {
		lck_mtx_unlock(&qrtr_lock);
		return 0;
	}
	qrtr_started = true;
	lck_mtx_unlock(&qrtr_lock);

	PE_parse_boot_argn("apollo_qrtr_debug", &qrtr_debug, sizeof(qrtr_debug));
	if (kernel_thread_start(qrtr_fresh_thread, NULL, &thread) != KERN_SUCCESS) {
		say("apollo qrtr: no thread for lookups\n");
		qrtr_started = false;
		return ENOMEM;
	}
	thread_set_thread_name(thread, "apollo qrtr");
	thread_deallocate(thread);
	apollo_ipcr_set_receive(qrtr_receive, NULL);    /* first: the chip may answer at once */
	/* what qrtr_endpoint_register() has Linux do: HELLO, from {node 1, control port} to {the node, control port} */
	error = qrtr_out_packet(QRTR_HELLO, QRTR_CONTROL, QRTR_NODE_UNKNOWN, QRTR_CONTROL, &hello, sizeof(hello));
	say("apollo qrtr: HELLO to the chip: %s\n", error == 0 ? "sent" : "NOT SENT");
	hakios_qmi_set_transport(&qrtr_ops);
	return error;
}
