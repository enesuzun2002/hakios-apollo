// census blob tsv : per-image counts of instruction classes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
enum { C_PACHINT, C_BTI, C_RETA, C_BRA, C_BLRA, C_ERETA, C_LDRA, C_PACDP, C_XPAC, C_PACGA, C_FJCVTZS, C_LDAPR, C_LDAPUR, C_MTE, C_FRINT, C_SB, C_CFINV, C_BF16I8MM, C_LD64B, C_WFXT, C_CSSC, C_OTHERHINT, C_NOP, N };
static const char *nm[N] = {"hint-PAC","BTI","RETAA/B","BRAA*","BLRAA*","ERETAA/B","LDRAA/B","PAC/AUT reg","XPACI/D","PACGA","FJCVTZS","LDAPR","LDAPUR/STLUR","MTE","FRINT32/64","SB","CFINV/AXFLAG","BF16/I8MM","LD64B/ST64B","WFET/WFIT","CSSC","other-hint","NOP"};
static int cls(uint32_t w) {
  if ((w & 0xFFFFF01F) == 0xD503201F) {
    uint32_t imm = (w >> 5) & 0x7F;
    if (imm == 0) return C_NOP;
    if (imm >= 0x20 && imm <= 0x27 && (w & 0xFFFFFF3F) == 0xD503241F) return C_BTI; // hint 32..39 (BTI variants: CRm=4 op2 0,2,4,6)
    if (imm == 7 || imm == 8 || imm == 10 || imm == 12 || imm == 14 || (imm >= 0x18 && imm <= 0x1F)) return C_PACHINT;
    return C_OTHERHINT;
  }
  if (w == 0xD65F0BFF || w == 0xD65F0FFF) return C_RETA;
  if (w == 0xD69F0BFF || w == 0xD69F0FFF) return C_ERETA;
  if ((w & 0xFFFFF81F) == 0xD61F081F - 0x1F + 0x1F && 0) return 0;
  if ((w & 0xFFFFF81F) == 0xD61F081F) return C_BRA; // BRAAZ/BRABZ: M bit 10
  if ((w & 0xFFFFF800) == 0xD71F0800) return C_BRA; // BRAA/BRAB
  if ((w & 0xFFFFF81F) == 0xD63F081F) return C_BLRA;
  if ((w & 0xFFFFF800) == 0xD73F0800) return C_BLRA;
  if ((w & 0xFF200400) == 0xF8200400) return C_LDRA;
  if ((w & 0xFFFFC000) == 0xDAC10000) return C_PACDP;
  if ((w & 0xFFFFF800) == 0xDAC14000) return C_XPAC;
  if ((w & 0xFFE0FC00) == 0x9AC03000) return C_PACGA;
  if ((w & 0xFFFFFC00) == 0x1E7E0000) return C_FJCVTZS;
  if ((w & 0x3FFFFC00) == 0x38BFC000) return C_LDAPR;
  if ((w & 0x3F200C00) == 0x19000000) return C_LDAPUR;
  if ((w & 0xFFE0FC00) == 0x9AC01000 || (w & 0xFFE0FC00) == 0x9AC01400) return C_MTE; // IRG, GMI
  if ((w & 0xFFC0C000) == 0x91800000 || (w & 0xFFC0C000) == 0xD1800000) return C_MTE; // ADDG SUBG
  if ((w & 0xFF200000) == 0xD9200000) return C_MTE; // STG/STZG/ST2G/STZ2G/LDG/STGM/LDGM...
  if ((w & 0xFE400000) == 0x68000000 && ((w>>23)&3) != 0 && ((w>>26)&1)==0) return C_MTE; // STGP (pre/post/off)
  if ((w & 0xFFBC7C00) == 0x1E284000) return C_FRINT;
  if ((w & 0x9FBFEC00) == 0x0E21E800) return C_FRINT;
  if (w == 0xD50330FF) return C_SB;
  if (w == 0xD500401F || w == 0xD500403F) return C_CFINV; // CFINV, XAFLAG D500403F, AXFLAG D500405F
  if (w == 0xD500405F) return C_CFINV;
  if ((w & 0xFFE0FC00) == 0xFFFFFFFF) return 0;
  // BF16/I8MM: BFDOT vec 0x2E40FC00 (mask), BFMMLA 0x6E40EC00, BFMLALB/T 0x2EC0FC00, BFCVT 0x1E634000, SMMLA 0x4E80A400, UMMLA 0x6E80A400, USMMLA 0x4E80AC00, USDOT 0x0E809C00, SUDOT/USDOT indexed
  if ((w & 0xBFE0FC00) == 0x0E40FC00 && ((w&0x20000000)!=0)) return C_BF16I8MM; // BFDOT (vector) U=1 ... approx
  if ((w & 0xFFE0FC00) == 0x6E40EC00) return C_BF16I8MM; // BFMMLA
  if ((w & 0xFFE0FC00) == 0x4E80A400 || (w & 0xFFE0FC00) == 0x6E80A400 || (w & 0xFFE0FC00) == 0x4E80AC00) return C_BF16I8MM;
  if ((w & 0xBFE0FC00) == 0x0E809C00) return C_BF16I8MM; // USDOT vec
  if ((w & 0xFFFFFC00) == 0x1E634000 || (w & 0xFFFFFC00) == 0x1E6340000) return C_BF16I8MM; // BFCVT (not exact; see llvm pass)
  if ((w & 0xFFFFFC00) == 0xF83FD000 - 0 && 0) return 0;
  if ((w & 0xFFFFFC00) == 0xF83FD000) return C_LD64B; // LD64B Xt,[Xn]
  if ((w & 0xFFFFFC00) == 0xF83F9000 || (w & 0xFFFFFC00) == 0xF83FB000) return C_LD64B; // ST64B / ST64BV
  if ((w & 0xFFFFFFE0) == 0xD5031000) return C_WFXT; // WFET
  if ((w & 0xFFFFFFE0) == 0xD5031020) return C_WFXT; // WFIT
  return -1;
}
int main(int argc, char **argv) {
  FILE *f = fopen(argv[1], "rb"); fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
  uint32_t *b = malloc(n); if (fread(b, 1, n, f) != n) return 1;
  FILE *t = fopen(argv[2], "r"); char line[4096];
  static unsigned long tot[N]; static unsigned long all = 0;
  // image aggregation by name (consecutive sections of the same image are adjacent)
  char cur[4096] = ""; unsigned long c[N]; memset(c, 0, sizeof c); unsigned long words = 0;
  FILE *o = fopen(argv[3], "w");
  #define FLUSH() if (cur[0]) { fprintf(o, "%s\t%lu", cur, words); for (int i = 0; i < N; i++) fprintf(o, "\t%lu", c[i]); fprintf(o, "\n"); }
  while (fgets(line, sizeof line, t)) {
    char *nmx = strtok(line, "\t"); long st = atol(strtok(NULL, "\t")); long ln = atol(strtok(NULL, "\t"));
    if (strcmp(nmx, cur)) { FLUSH(); strcpy(cur, nmx); memset(c, 0, sizeof c); words = 0; }
    words += ln / 4;
    for (long i = st / 4; i < (st + ln) / 4; i++) { int k = cls(b[i]); if (k >= 0) { c[k]++; tot[k]++; } }
    all += ln / 4;
  }
  FLUSH();
  fprintf(o, "#header\twords");
  for (int i = 0; i < N; i++) fprintf(o, "\t%s", nm[i]); fprintf(o, "\n");
  printf("total words %lu\n", all); for (int i = 0; i < N; i++) printf("%-16s %lu\n", nm[i], tot[i]);
}
