/*
 * ladder.c - generate ASCII ladder-logic diagrams from state tables.
 *
 * INPUT
 *   One or more relay definitions.  A line containing "(name)" starts a relay;
 *   the contact lines after it (up to the next "(name)" line or EOF) are that
 *   relay's table.  Contact lines before any "(name)" belong to relay "r1".
 *
 *       (r1)
 *       D    NEXT + 2    NEXT         # D on: bypass A and B, go to C
 *       A    NEXT        OFF
 *       B    NEXT        OFF    P     # optional 3rd column defines label P
 *       C    ON          OFF
 *
 *       (r2)
 *       E    P           OFF          # E on: continue at P (B, then C)
 *
 *   Contact line:  NAME  <ON action>  <OFF action>  [label]
 *   Actions:       ON | OFF | NEXT | NEXT + n | <label>
 *   '#' starts a comment.  A line whose first word is ON/OFF/NEXT (the
 *   "ON OFF" column header) is ignored.  Names are case-insensitive and are
 *   printed in lower case.
 *
 * HOW IT WORKS
 *   1. Parse, then resolve every action to a target (ON, OFF, or an entry).
 *      A label may point into another relay's entries; each relay is drawn
 *      from the entries reachable from its own first entry, so the reused
 *      contacts are copied into that relay's rung.
 *   2. Compute each entry's truth table.  For a contact, the exit whose
 *      result contains the other's is the "pass" path (ON exit = normally
 *      open [ ], OFF exit = normally closed [/]); the other is the "fail"
 *      path, i.e. the next parallel alternative.
 *   3. Recover the series/parallel tree recursively.  Entries whose fail
 *      path equals the enclosing alternative are series contacts; any other
 *      fail path opens a parallel group that closes where the pass paths of
 *      its branches meet.
 *   4. Check the tree against the table for every input combination, then
 *      lay it out into a character grid.
 *
 * USAGE: ladder [-t] [table_file]      (stdin if no file)
 *   -t  draw every series left-to-right in table order.  The default mirrors
 *       the order of every series (table S2,S1 -> diagram s1,s2), i.e. the
 *       table is read from the coil toward the left rail.
 *
 * Build: cc -Wall -O2 -o ladder ladder.c
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <stdint.h>
#include <stdarg.h>

#define MAX_ENTRIES 512
#define MAX_RELAYS  64
#define MAX_NAME    32
#define MAX_VARS    20          /* distinct inputs per relay (truth tables) */

#define TGT_ON   (-1)
#define TGT_OFF  (-2)

#define GAP        6            /* wire between series items          */
#define RAIL_LEAD  7            /* wire between '>' and first contact */
#define LPAD       3            /* wire after a nested group's left junction */
#define RPAD       4            /* wire before a group's right junction */
#define BLANK      '\001'       /* opaque space inside [ ] */

typedef enum { A_ON, A_OFF, A_NEXT, A_LABEL } ActKind;
typedef struct { ActKind kind; int skip; char label[MAX_NAME]; } Action;

typedef struct {
    char   name[MAX_NAME];
    char   label[MAX_NAME];            /* label defined here ("" = none) */
    Action on, off;
    int    relay, line;
    int    on_t, off_t;                /* resolved targets               */
    int    pass_t, fail_t, nc;         /* classified for the current relay */
    int    reached;                    /* reachable from some relay      */
} Entry;

typedef struct { char name[MAX_NAME]; int start, end; } Relay;

static Entry ent[MAX_ENTRIES];
static int   nent;
static Relay rel[MAX_RELAYS];
static int   nrel;
static char  errmsg[512];
static int   g_reverse = 1;

static void *fail(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(errmsg, sizeof errmsg, fmt, ap);
    va_end(ap);
    return NULL;
}

/* ================================ parsing ================================ */

static void lowername(char *dst, const char *src, size_t n)
{
    size_t i;
    for (i = 0; src[i] && i < n - 1; i++) dst[i] = (char)tolower((unsigned char)src[i]);
    dst[i] = '\0';
}

static int is_kw(const char *t)
{
    return !strcasecmp(t, "ON") || !strcasecmp(t, "OFF") || !strcasecmp(t, "NEXT");
}

static int parse_one(char **tok, int *pos, int nt, Action *a)
{
    if (*pos >= nt) return 0;
    char *t = tok[(*pos)++];
    a->skip = 0;
    a->label[0] = '\0';
    if (!strcasecmp(t, "ON"))  { a->kind = A_ON;  return 1; }
    if (!strcasecmp(t, "OFF")) { a->kind = A_OFF; return 1; }
    if (!strcasecmp(t, "NEXT")) {
        a->kind = A_NEXT;
        if (*pos < nt && tok[*pos][0] == '+') {
            (*pos)++;
            if (*pos >= nt) return 0;
            a->skip = atoi(tok[(*pos)++]);
            if (a->skip < 0) return 0;
        }
        return 1;
    }
    if (t[0] == '+') return 0;
    a->kind = A_LABEL;
    lowername(a->label, t, MAX_NAME);
    return 1;
}

/* 1 = contact, 2 = relay line, 0 = skipped, -1 = syntax error */
static int parse_line(char *line, Entry *e, char *relay_name)
{
    char buf[1024], *tok[32];
    size_t j = 0;
    int nt = 0;

    char *hash = strchr(line, '#');
    if (hash) *hash = '\0';

    char *p = line;
    while (isspace((unsigned char)*p)) p++;
    if (*p == '(') {
        char *close = strchr(p, ')');
        if (!close) return -1;
        *close = '\0';
        p++;
        while (isspace((unsigned char)*p)) p++;
        char *q = p + strlen(p);
        while (q > p && isspace((unsigned char)q[-1])) *--q = '\0';
        if (!*p) return -1;
        lowername(relay_name, p, MAX_NAME);
        return 2;
    }

    for (const char *s = line; *s && j < sizeof buf - 4; s++) {
        if (*s == '+') { buf[j++] = ' '; buf[j++] = '+'; buf[j++] = ' '; }
        else           buf[j++] = *s;
    }
    buf[j] = '\0';
    for (char *t = strtok(buf, " \t\r\n"); t && nt < 32; t = strtok(NULL, " \t\r\n"))
        tok[nt++] = t;

    if (nt == 0) return 0;
    if (is_kw(tok[0])) return 0;                      /* column header */

    int pos = 1;
    if (!parse_one(tok, &pos, nt, &e->on))  return -1;
    if (!parse_one(tok, &pos, nt, &e->off)) return -1;
    if (pos < nt) {                                   /* optional label */
        if (is_kw(tok[pos])) return -1;
        lowername(e->label, tok[pos], MAX_NAME);
        pos++;
    }
    if (pos != nt) return -1;
    lowername(e->name, tok[0], MAX_NAME);
    return 1;
}

static int new_relay(const char *name)
{
    if (nrel >= MAX_RELAYS) { fprintf(stderr, "too many relays\n"); return -1; }
    for (int i = 0; i < nrel; i++)
        if (!strcmp(rel[i].name, name)) {
            fprintf(stderr, "relay (%s) is defined twice\n", name);
            return -1;
        }
    if (nrel) rel[nrel - 1].end = nent;
    strcpy(rel[nrel].name, name);
    rel[nrel].start = nent;
    nrel++;
    return 0;
}

static int parse_table(FILE *fp)
{
    char line[256], rname[MAX_NAME];
    int lineno = 0;
    Entry tmp;

    while (fgets(line, sizeof line, fp)) {
        lineno++;
        memset(&tmp, 0, sizeof tmp);
        int r = parse_line(line, &tmp, rname);
        if (r < 0) { fprintf(stderr, "line %d: syntax error\n", lineno); return -1; }
        if (r == 2) { if (new_relay(rname) < 0) return -1; continue; }
        if (r == 1) {
            if (nent >= MAX_ENTRIES) { fprintf(stderr, "too many entries\n"); return -1; }
            if (nrel == 0 && new_relay("r1") < 0) return -1;
            tmp.relay = nrel - 1;
            tmp.line  = lineno;
            ent[nent++] = tmp;
        }
    }
    if (nrel == 0) return -1;
    rel[nrel - 1].end = nent;
    return 0;
}

static int find_label(const char *nm)
{
    for (int i = 0; i < nent; i++)
        if (ent[i].label[0] && !strcmp(ent[i].label, nm)) return i;
    return -1;
}

static int resolve_all(void)
{
    for (int i = 0; i < nent; i++)
        for (int j = i + 1; j < nent; j++)
            if (ent[i].label[0] && !strcmp(ent[i].label, ent[j].label)) {
                fprintf(stderr, "line %d: label '%s' already defined on line %d\n",
                        ent[j].line, ent[j].label, ent[i].line);
                return -1;
            }

    for (int i = 0; i < nent; i++) {
        const Relay *R = &rel[ent[i].relay];
        for (int k = 0; k < 2; k++) {
            const Action *a = k ? &ent[i].off : &ent[i].on;
            int t = TGT_OFF;
            switch (a->kind) {
            case A_ON:  t = TGT_ON;  break;
            case A_OFF: t = TGT_OFF; break;
            case A_NEXT:
                t = i + 1 + a->skip;
                if (t == R->end) t = TGT_OFF;          /* falls off the end */
                else if (t > R->end) {
                    fprintf(stderr, "line %d: NEXT + %d jumps past the end of relay (%s)\n",
                            ent[i].line, a->skip, R->name);
                    return -1;
                }
                break;
            case A_LABEL:
                t = find_label(a->label);
                if (t < 0) {
                    fprintf(stderr, "line %d: undefined label '%s'\n", ent[i].line, a->label);
                    return -1;
                }
                break;
            }
            if (k) ent[i].off_t = t; else ent[i].on_t = t;
        }
    }
    return 0;
}

/* ============================ circuit recovery =========================== */

typedef enum { N_CONTACT, N_SERIES, N_PARALLEL } NType;
typedef struct Node {
    int type, entry, nkid, cap;
    struct Node **kid;
} Node;

static Node *node_new(int type)
{
    Node *n = calloc(1, sizeof *n);
    n->type = type;
    n->entry = -1;
    return n;
}

static void node_add(Node *n, Node *k)            /* flattens same-type kids */
{
    if (k->type == n->type && k->type != N_CONTACT) {
        for (int i = 0; i < k->nkid; i++) node_add(n, k->kid[i]);
        free(k->kid);
        free(k);
        return;
    }
    if (n->nkid == n->cap) {
        n->cap = n->cap ? n->cap * 2 : 4;
        n->kid = realloc(n->kid, (size_t)n->cap * sizeof *n->kid);
    }
    n->kid[n->nkid++] = k;
}

static Node *node_done(Node *n)
{
    if (n->nkid == 0) { free(n); return NULL; }
    if (n->nkid == 1) { Node *k = n->kid[0]; free(n->kid); free(n); return k; }
    return n;
}

/* ---- per-relay working state ---- */
static int       rstate[MAX_ENTRIES];     /* 0 new, 1 open, 2 done */
static int       order[MAX_ENTRIES], norder;
static int       var_of[MAX_ENTRIES];
static char      vname[MAX_VARS + 1][MAX_NAME];
static int       nvars;
static int       W;                       /* words per truth table */
static uint64_t *Vv[MAX_VARS], *ALLB, *ZEROB, *fn[MAX_ENTRIES];
static int       pass_mark[MAX_ENTRIES], mark_stamp;

static int dfs(int e)
{
    if (rstate[e] == 2) return 0;
    if (rstate[e] == 1) {
        fail("jump loop through entry '%s' (line %d)", ent[e].name, ent[e].line);
        return -1;
    }
    rstate[e] = 1;
    ent[e].reached = 1;
    if (ent[e].on_t  >= 0 && dfs(ent[e].on_t)  < 0) return -1;
    if (ent[e].off_t >= 0 && dfs(ent[e].off_t) < 0) return -1;
    rstate[e] = 2;
    order[norder++] = e;
    return 0;
}

static uint64_t *tfn(int t)
{
    return t == TGT_ON ? ALLB : t == TGT_OFF ? ZEROB : fn[t];
}

static int subset(const uint64_t *a, const uint64_t *b)   /* a implies b */
{
    for (int w = 0; w < W; w++) if (a[w] & ~b[w]) return 0;
    return 1;
}

/*
 * Pass-path bookkeeping.  mark_path() numbers the nodes on the pass-path of
 * `a` (the coil, TGT_ON, gets the last number).  join_pos() returns the
 * number of the first node on b's pass-path that also lies on a's.
 */
static int pass_pos[MAX_ENTRIES], path_len;

static void mark_path(int a)
{
    mark_stamp++;
    int i = 0;
    for (int p = a; p >= 0; p = ent[p].pass_t) { pass_mark[p] = mark_stamp; pass_pos[p] = i++; }
    path_len = i;
}

static int join_node(int b, int *pos)
{
    for (int p = b;; p = ent[p].pass_t) {
        if (p < 0) { *pos = path_len; return TGT_ON; }
        if (pass_mark[p] == mark_stamp) { *pos = pass_pos[p]; return p; }
    }
}

static Node *contact_node(int e)
{
    Node *n = node_new(N_CONTACT);
    n->entry = e;
    return n;
}

/*
 * Series run from entry `cur` up to (not including) join `j`.  `r` is the
 * alternative the enclosing context falls back to when a contact fails.
 *
 * A contact whose fail path is `r` is a plain series contact.  Otherwise it
 * heads a parallel group.  Following the fail pointers from `cur` (until `r`)
 * lists every alternative that could belong to a group opening here; the
 * outermost group is the one whose branches rejoin furthest along cur's
 * pass-path (without passing `j`), and the nearest such alternative is taken.
 */
static Node *build_seq(int cur, int j, int r)
{
    Node *ser = node_new(N_SERIES);

    while (cur != j) {
        if (cur < 0)
            return fail("paths do not rejoin where expected - not a series-parallel network");
        Entry *e = &ent[cur];

        if (e->fail_t == r) {                         /* series contact */
            node_add(ser, contact_node(cur));
            cur = e->pass_t;
            continue;
        }

        mark_path(cur);
        int jpos = (j < 0) ? path_len : (pass_mark[j] == mark_stamp ? pass_pos[j] : -1);
        if (jpos < 0)
            return fail("entry '%s' (line %d): paths do not rejoin where expected - "
                        "not a series-parallel network", e->name, e->line);

        int alt = -1, best = -1;
        for (int h = e->fail_t; h != r; h = ent[h].fail_t) {
            if (h < 0)
                return fail("entry '%s' (line %d): failing path goes to OFF, but a parallel "
                            "alternative is still pending - not a series-parallel network",
                            e->name, e->line);
            int pos;
            join_node(h, &pos);
            if (pos <= jpos && pos > best) { best = pos; alt = h; }
        }
        if (alt < 0)
            return fail("entry '%s' (line %d): parallel group does not nest inside its "
                        "enclosing group - not a series-parallel network", e->name, e->line);

        int j2 = join_node(alt, &best);
        if (j2 == alt)
            return fail("entry '%s' (line %d): contact is bypassed by a plain wire - "
                        "it has no effect", e->name, e->line);

        Node *b1 = build_seq(cur, j2, alt);
        if (!b1) return NULL;
        Node *b2 = build_seq(alt, j2, r);
        if (!b2) return NULL;
        Node *par = node_new(N_PARALLEL);
        node_add(par, b1);
        node_add(par, b2);
        node_add(ser, par);
        cur = j2;
    }
    Node *res = node_done(ser);
    if (!res) return fail("empty branch");
    return res;
}

static uint64_t *tree_fn(const Node *n)
{
    uint64_t *r = calloc((size_t)W, sizeof *r);
    if (n->type == N_CONTACT) {
        const uint64_t *v = Vv[var_of[n->entry]];
        for (int w = 0; w < W; w++) r[w] = ent[n->entry].nc ? (~v[w] & ALLB[w]) : v[w];
    } else {
        int ser = n->type == N_SERIES;
        for (int w = 0; w < W; w++) r[w] = ser ? ALLB[w] : 0;
        for (int i = 0; i < n->nkid; i++) {
            uint64_t *t = tree_fn(n->kid[i]);
            for (int w = 0; w < W; w++) r[w] = ser ? (r[w] & t[w]) : (r[w] | t[w]);
            free(t);
        }
    }
    return r;
}

static void free_tables(void)
{
    for (int v = 0; v < MAX_VARS; v++) { free(Vv[v]); Vv[v] = NULL; }
    free(ALLB);  ALLB = NULL;
    free(ZEROB); ZEROB = NULL;
    for (int i = 0; i < MAX_ENTRIES; i++) { free(fn[i]); fn[i] = NULL; }
}

static Node *build_relay(int ri)
{
    const Relay *R = &rel[ri];
    Node *tree = NULL;

    if (R->start == R->end) return fail("no entries");

    memset(rstate, 0, sizeof rstate);
    norder = 0;
    if (dfs(R->start) < 0) return NULL;

    /* distinct inputs */
    nvars = 0;
    for (int k = 0; k < norder; k++) {
        int e = order[k], v;
        for (v = 0; v < nvars; v++) if (!strcmp(vname[v], ent[e].name)) break;
        if (v == nvars) {
            if (nvars >= MAX_VARS)
                return fail("more than %d distinct inputs (MAX_VARS)", MAX_VARS);
            strcpy(vname[nvars++], ent[e].name);
        }
        var_of[e] = v;
    }

    /* truth tables */
    size_t nbits = (size_t)1 << nvars;
    W = (int)((nbits + 63) / 64);
    ALLB  = calloc((size_t)W, sizeof *ALLB);
    ZEROB = calloc((size_t)W, sizeof *ZEROB);
    for (int v = 0; v < nvars; v++) Vv[v] = calloc((size_t)W, sizeof **Vv);
    for (size_t a = 0; a < nbits; a++) {
        ALLB[a >> 6] |= 1ULL << (a & 63);
        for (int v = 0; v < nvars; v++)
            if ((a >> v) & 1) Vv[v][a >> 6] |= 1ULL << (a & 63);
    }
    for (int k = 0; k < norder; k++) {
        int e = order[k];
        const uint64_t *X = tfn(ent[e].on_t), *Y = tfn(ent[e].off_t), *V = Vv[var_of[e]];
        fn[e] = calloc((size_t)W, sizeof **fn);
        for (int w = 0; w < W; w++)
            fn[e][w] = ((V[w] & X[w]) | (~V[w] & Y[w])) & ALLB[w];
    }

    /* classify each contact: which exit passes, which falls through */
    for (int k = 0; k < norder; k++) {
        Entry *x = &ent[order[k]];
        if (x->on_t == x->off_t) {
            if (x->on_t == TGT_OFF)
                fail("entry '%s' (line %d): both actions end in OFF, so this contact can "
                     "never turn the relay on (does the table end before reaching an ON?)",
                     x->name, x->line);
            else
                fail("entry '%s' (line %d): both actions lead to the same place - "
                     "the contact has no effect", x->name, x->line);
            goto done;
        }
        const uint64_t *X = tfn(x->on_t), *Y = tfn(x->off_t);
        if (subset(Y, X))      { x->pass_t = x->on_t;  x->fail_t = x->off_t; x->nc = 0; }
        else if (subset(X, Y)) { x->pass_t = x->off_t; x->fail_t = x->on_t;  x->nc = 1; }
        else {
            fail("entry '%s' (line %d): neither exit's result contains the other's, so it "
                 "is not a plain contact in a series-parallel network (the table selects "
                 "between two different continuations)", x->name, x->line);
            goto done;
        }
        if (x->pass_t == TGT_OFF) {
            fail("entry '%s' (line %d): the conducting path leads to OFF", x->name, x->line);
            goto done;
        }
        if (x->fail_t == TGT_ON) {
            fail("entry '%s' (line %d): the non-conducting path leads to ON", x->name, x->line);
            goto done;
        }
    }

    tree = build_seq(R->start, TGT_ON, TGT_OFF);
    if (!tree) goto done;

    /* the drawing must compute exactly what the table computes */
    {
        uint64_t *tf = tree_fn(tree);
        const uint64_t *want = fn[R->start];
        for (int w = 0; w < W; w++) {
            uint64_t d = tf[w] ^ want[w];
            if (!d) continue;
            int bit = __builtin_ctzll(d);
            size_t a = (size_t)w * 64 + (size_t)bit;
            char in[512] = "";
            for (int v = 0; v < nvars; v++) {
                char one[48];
                snprintf(one, sizeof one, " %s=%d", vname[v], (int)((a >> v) & 1));
                strncat(in, one, sizeof in - strlen(in) - 1);
            }
            fail("table is not a series-parallel circuit (with inputs%s the table gives %s "
                 "but the recovered circuit gives %s) - check the fail routing",
                 in, (want[w] >> bit) & 1 ? "ON" : "OFF", (tf[w] >> bit) & 1 ? "ON" : "OFF");
            tree = NULL;
            break;
        }
        free(tf);
    }

done:
    free_tables();
    return tree;
}

/* ================================ layout ================================= */

typedef struct { int w, h, m; char *c; } Canvas;       /* m = main wire row */

static Canvas *cv_new(int w, int h, int m)
{
    Canvas *k = malloc(sizeof *k);
    k->w = w; k->h = h; k->m = m;
    k->c = malloc((size_t)w * (size_t)h);
    memset(k->c, ' ', (size_t)w * (size_t)h);
    return k;
}
static void cv_put(Canvas *k, int r, int c, char ch)
{
    if (r >= 0 && r < k->h && c >= 0 && c < k->w) k->c[r * k->w + c] = ch;
}
static void cv_blit(Canvas *d, const Canvas *s, int r0, int c0)
{
    for (int r = 0; r < s->h; r++)
        for (int c = 0; c < s->w; c++) {
            char ch = s->c[r * s->w + c];
            if (ch != ' ') cv_put(d, r0 + r, c0 + c, ch);
        }
}
static void cv_free(Canvas *k) { free(k->c); free(k); }

static Canvas *layout(const Node *n, int rail);

static Canvas *lay_contact(const Node *n)
{
    const char *nm = ent[n->entry].name;
    int len = (int)strlen(nm);
    int w = len + 1 > 3 ? len + 1 : 3;
    Canvas *k = cv_new(w, 2, 1);
    for (int c = 0; c < w; c++) cv_put(k, 1, c, '-');
    cv_put(k, 1, 0, '[');
    cv_put(k, 1, 1, ent[n->entry].nc ? '/' : BLANK);
    cv_put(k, 1, 2, ']');
    for (int i = 0; i < len; i++) cv_put(k, 0, 1 + i, nm[i]);
    return k;
}

static Canvas *lay_series(const Node *n, int rail)
{
    int nk = n->nkid, above = 0, below = 0, w = 0;
    Canvas **k = malloc((size_t)nk * sizeof *k);
    int *x = malloc((size_t)nk * sizeof *x);

    for (int i = 0; i < nk; i++) {
        k[i] = layout(n->kid[i], rail && i == 0);
        if (k[i]->m > above) above = k[i]->m;
        int b = k[i]->h - k[i]->m - 1;
        if (b > below) below = b;
        x[i] = w;
        w += k[i]->w + (i < nk - 1 ? GAP : 0);
    }
    Canvas *r = cv_new(w, above + 1 + below, above);
    for (int c = 0; c < w; c++) cv_put(r, above, c, '-');
    for (int i = 0; i < nk; i++) {
        cv_blit(r, k[i], above - k[i]->m, x[i]);
        cv_free(k[i]);
    }
    free(k);
    free(x);
    return r;
}

static Canvas *lay_parallel(const Node *n, int rail)
{
    int nb = n->nkid, iw = 0, H = 0;
    Canvas **b = malloc((size_t)nb * sizeof *b);
    int *y = malloc((size_t)nb * sizeof *y);

    for (int i = 0; i < nb; i++) {
        b[i] = layout(n->kid[i], 0);
        if (b[i]->w > iw) iw = b[i]->w;
    }
    int lpad = rail ? RAIL_LEAD : LPAD;
    int Wd = 1 + lpad + iw + RPAD + 1;
    for (int i = 0; i < nb; i++) { y[i] = H; H += b[i]->h + (i < nb - 1 ? 1 : 0); }

    Canvas *k = cv_new(Wd, H, b[0]->m);
    int r0 = y[0] + b[0]->m, rl = y[nb - 1] + b[nb - 1]->m;

    for (int r = r0 + 1; r < rl; r++) {
        if (!rail) cv_put(k, r, 0, '|');
        cv_put(k, r, Wd - 1, '|');
    }
    for (int i = 0; i < nb; i++) {
        int rr = y[i] + b[i]->m;
        for (int c = 1; c < Wd - 1; c++) cv_put(k, rr, c, '-');
        cv_blit(k, b[i], y[i], 1 + lpad);
        cv_put(k, rr, 0, rail ? '>' : '+');
        cv_put(k, rr, Wd - 1, '+');
        cv_free(b[i]);
    }
    free(b);
    free(y);
    return k;
}

static Canvas *layout(const Node *n, int rail)
{
    switch (n->type) {
    case N_CONTACT: return lay_contact(n);
    case N_SERIES:  return lay_series(n, rail);
    default:        return lay_parallel(n, rail);
    }
}

static void reverse_series(Node *n)
{
    if (n->type == N_CONTACT) return;
    if (n->type == N_SERIES)
        for (int a = 0, b = n->nkid - 1; a < b; a++, b--) {
            Node *t = n->kid[a]; n->kid[a] = n->kid[b]; n->kid[b] = t;
        }
    for (int i = 0; i < n->nkid; i++) reverse_series(n->kid[i]);
}

static void render_relay(Node *root, const char *name)
{
    Node wrap, *one[1], *ser = root;

    if (g_reverse) reverse_series(root);
    if (root->type != N_SERIES) {
        memset(&wrap, 0, sizeof wrap);
        wrap.type = N_SERIES; wrap.nkid = wrap.cap = 1; wrap.kid = one; wrap.entry = -1;
        one[0] = root;
        ser = &wrap;
    }
    int first_par = ser->kid[0]->type == N_PARALLEL;
    int last_par  = ser->kid[ser->nkid - 1]->type == N_PARALLEL;

    Canvas *body = layout(ser, 1);
    int lead = first_par ? 0 : 1 + RAIL_LEAD;
    int tail = last_par ? 3 : 8;
    char coil[MAX_NAME + 8];
    snprintf(coil, sizeof coil, "(%.*s)--", MAX_NAME - 1, name);

    int Wd = lead + body->w + tail + (int)strlen(coil);
    Canvas *k = cv_new(Wd, body->h, body->m);
    for (int c = 0; c < Wd; c++) cv_put(k, body->m, c, '-');
    if (!first_par) cv_put(k, body->m, 0, '>');
    cv_blit(k, body, 0, lead);
    for (int i = 0; coil[i]; i++) cv_put(k, body->m, lead + body->w + tail + i, coil[i]);

    for (int r = 0; r < k->h; r++) {
        int len = k->w;
        while (len > 0 && k->c[r * k->w + len - 1] == ' ') len--;
        for (int c = 0; c < len; c++) {
            char ch = k->c[r * k->w + c];
            putchar(ch == BLANK ? ' ' : ch);
        }
        putchar('\n');
    }
    cv_free(body);
    cv_free(k);
}

/* ================================== main ================================= */

int main(int argc, char **argv)
{
    const char *path = NULL;
    Node *tree[MAX_RELAYS];

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-t")) g_reverse = 0;
        else path = argv[i];
    }

    FILE *fp = path ? fopen(path, "r") : stdin;
    if (!fp) { perror(path); return 1; }
    if (parse_table(fp) < 0) { fprintf(stderr, "no valid table\n"); return 1; }
    if (fp != stdin) fclose(fp);
    if (resolve_all() < 0) return 1;

    for (int i = 0; i < nrel; i++) {
        tree[i] = build_relay(i);
        if (!tree[i]) { fprintf(stderr, "relay (%s): %s\n", rel[i].name, errmsg); return 1; }
    }
    for (int i = 0; i < nent; i++)
        if (!ent[i].reached)
            fprintf(stderr, "warning: line %d: entry '%s' is never reached\n",
                    ent[i].line, ent[i].name);

    for (int i = 0; i < nrel; i++) {
        if (i) putchar('\n');
        render_relay(tree[i], rel[i].name);
    }
    return 0;
}
