# ladder

Generates ASCII ladder-logic wiring diagrams from a state table.

`ladder` reads a table of contacts, each with an ON action and an OFF action,
recovers the series/parallel circuit the table describes, verifies that circuit
against the table, and prints it as a plain-text ladder rung.

## Features

- One rung per relay; any number of relays per file
- Normally open `[ ]` and normally closed `[/]` contacts, inferred from the table
- Nested series/parallel circuits, including bypass contacts
- Entry points (labels) so one relay can reuse contacts from another
- Self-check: the recovered circuit is compared to the table for every input
  combination, and the program fails with a message if they differ

## Build

```
cc -Wall -O2 -o ladder ladder.c
```

Plain C99, no dependencies. Uses `__builtin_ctzll` (GCC/Clang).

## Usage

```
ladder [-t] [table_file]
```

Reads `table_file`, or stdin if none is given. Diagrams go to stdout; errors and
warnings go to stderr.

| Option | Effect |
|--------|--------|
| *(none)* | Mirror the order of every series: the table is read from the coil toward the left rail (table `S2,S1` draws as `s1,s2`) |
| `-t` | Draw every series left to right in table order |

## Input format

```
(r1)
D    NEXT + 2    NEXT
A    NEXT        OFF
B    NEXT        OFF      P
C    ON          OFF
```

A line starting with `(name)` begins a relay. The contact lines that follow,
up to the next `(name)` line or end of file, are that relay's table. Contact
lines before any `(name)` line belong to relay `r1`.

Each contact line is:

```
NAME   <ON action>   <OFF action>   [label]
```

| Field | Meaning |
|-------|---------|
| `NAME` | Contact name (case-insensitive, printed in lower case) |
| ON action | Where to go when the contact is on |
| OFF action | Where to go when the contact is off |
| `label` | Optional third column; defines an entry point other lines can jump to |

### Actions

| Action | Meaning |
|--------|---------|
| `ON` | The relay is energized |
| `OFF` | The relay is not energized |
| `NEXT` | Continue with the next line |
| `NEXT + n` | Skip the next `n` lines, then continue |
| `<label>` | Continue at the line carrying that label, which may be in another relay |

### Other syntax

- `#` starts a comment.
- A line whose first word is `ON`, `OFF`, or `NEXT` (an `ON OFF` column header)
  is ignored.
- Labels are global across the file and must be unique.
- Relay names must be unique.

## Example

`table.txt`:

```
# bypass: D on skips A and B, so only C must be on
(r1)
D    NEXT + 2    NEXT
A    NEXT        OFF
B    NEXT        OFF      P     # P = entry point just before B
C    ON          OFF

# patch: E on continues at P (B, then C)
(r2)
E    P           OFF

# original example
(r3)
S2   NEXT + 1    NEXT
S1   ON          NEXT
S3   ON          OFF
```

`./ladder table.txt`:

```
         c            d
>-------[ ]------+---[ ]-------------+---(r1)--
                 |                   |
                 |    b        a     |
                 +---[ ]------[ ]----+

         c        b        e
>-------[ ]------[ ]------[ ]--------(r2)--

         s1       s2
>-------[ ]------[/]----+---(r3)--
                        |
         s3             |
>-------[ ]-------------+
```

`./ladder -t table.txt` (series in table order):

```
         d                      c
>-------[ ]-------------+------[ ]--------(r1)--
                        |
         a        b     |
>-------[ ]------[ ]----+

         e        b        c
>-------[ ]------[ ]------[ ]--------(r2)--

         s2       s1
>-------[/]------[ ]----+---(r3)--
                        |
         s3             |
>-------[ ]-------------+
```

### What the examples show

- **r1, bypass:** `D` on jumps straight to `C`, so `D` drawn in parallel with
  the `A`/`B` series leaves only `C` required.
- **r2, patch:** `E` on jumps to label `P`. `P` sits on `B`, so `r2` is drawn
  from the contacts reachable from its own first entry: `E`, then the shared
  `B` and `C`. The reused contacts are copied into `r2`'s rung.
- **r3, normally closed:** `S2` is drawn `[/]` because its OFF exit is the one
  that conducts.

## How it works

1. **Parse** relays and contact lines, then resolve every action to a target
   (`ON`, `OFF`, or another entry).
2. **Truth tables:** compute each entry's result over all input combinations.
   For each contact, the exit whose result contains the other's is the pass
   path. An ON-exit pass path is a normally open contact `[ ]`, an OFF-exit
   pass path is normally closed `[/]`. The other exit is the fail path, the
   next parallel alternative.
3. **Recover the tree:** entries whose fail path equals the enclosing
   alternative are series contacts; any other fail path opens a parallel group
   that closes where its branches' pass paths meet.
4. **Verify and draw:** check the tree against the table for every input
   combination, then lay it out on a character grid.

## Errors and warnings

The program exits with status 1 and a message on stderr for:

- syntax errors (`line N: syntax error`)
- duplicate relay names or duplicate labels
- an entry where neither exit's result contains the other's, meaning the table
  is not a plain contact in a series-parallel network
- a conducting path that leads to `OFF`, or a non-conducting path that leads to `ON`
- a table whose recovered circuit disagrees with the table; the message lists
  an input combination where they differ

An entry that no relay reaches produces a warning
(`line N: entry 'x' is never reached`) but does not stop output.

## Limits

| Limit | Value |
|-------|-------|
| Contact entries (total) | 512 |
| Relays | 64 |
| Name / label length | 31 characters |
| Distinct inputs per relay | 20 (truth-table size) |
| Tokens per line | 32 |
