#!/usr/bin/python
import sys

maph = None
mapw = None
vid = {}
obstacle = "SWT@O"
cw = 10 # cardinal weight
dw = 14 # diagonal weight
blocked_w = 100
dx = [0, 0, 1, -1, 1, -1, 1, -1]
dy = [-1, 1, 0, 0, -1, -1, 1, 1]
w  = [cw, cw, cw, cw, dw, dw, dw, dw]

def traversable(x: int, y: int) -> bool:
    if (x < 0 or x >= mapw or y < 0 or y >= maph
        or vid.get(y*mapw+x) is None):
        return False
    return True


def charmap2graph(mfile):
    with open(mfile, "r") as f:
        charmap = f.readlines()[4:]
    global maph, mapw
    maph = len(charmap)
    mapw = len(charmap[0])
    cnt = 0
    global vid
    for y in range(maph):
        for x in range(mapw):
            if (charmap[y][x] not in obstacle):
                vid[y*mapw+x] = cnt
                cnt += 1
    arcs = []
    for y in range(maph):
        for x in range(mapw):
            if (charmap[y][x] not in obstacle):
                u = vid[y*mapw+x]
                for i in range(len(dx)):
                    nx = x + dx[i]
                    ny = y + dy[i]
                    if (not traversable(nx, ny)): continue
                    v = vid[ny*mapw+nx]
                    arcs.append((u, v, w[i]))
    return charmap, arcs

def scen2queries(sfile):
    import pandas as pd
    header = ["bucket_id", "map", "h", "w", "sx", "sy", "tx", "ty", "ref_dist"]
    df: pd.DataFrame = pd.read_csv(sfile, skiprows=1, sep=r'\s+', header=0, names=header)
    queries = []
    for idx, row in df.iterrows():
        u = vid[row['sy']*mapw+row['sx']]
        v = vid[row['ty']*mapw+row['tx']]
        queries.append((u, v))
    return queries

def repair2diff(rfile: str, charmap: list[str]):
    with open(rfile, "r") as f:
        raw = f.readlines()
    num = int(raw[0])
    diff = []
    for row in raw[1:]:
        # (x, y) becomes blocked
        x, y, _, _, _ = map(int, row.split(' '))
        v = y*mapw+x
        for i in range(len(dx)):
            px = x - dx[i]
            py = y - dy[i]
            u = py*mapw+px
            if (not traversable(px, py)): continue
            diff.append((u, v, blocked_w))
            diff.append((v, u, blocked_w))
        vid.pop(v)
    return diff


def main(mfile, sfile, rfile, gfile=None, qfile=None, dfile=None):
    """
    mfile: grid map file, e.g. arena.map
    sfile: scen file, e.g. arena.map.scen
    cfile: repair file describes changing events, e.g.: arena.repair 
    write file to:
        * gfile: graph file in dimacs format
        * qfile: query file in dimacs format
        * dfile: edge change file in dimacs format
    """
    charmap, arcs = charmap2graph(mfile)
    queries = scen2queries(sfile)
    diff = repair2diff(rfile, charmap)

    if gfile is None: gfile = mfile + ".gr"
    if qfile is None: qfile = mfile + ".query"
    if dfile is None: dfile = mfile + ".diff"

    with open(gfile, "w") as f:
        f.write("p sp %d %d\n" % (len(vid.keys()), len(arcs)))
        for arc in arcs:
            f.write("a %d %d %d\n" % (arc[0], arc[1], arc[2]))

    with open(qfile, "w") as f:
        f.write("%d\n" % len(queries))
        for q in queries:
            f.write("%d %d\n" % (q[0], q[1]))

    with open(dfile, "w") as f:
        f.write("%d\n" % len(diff))
        for i in diff:
            f.write("%d %d %d\n" % (i[0], i[1], i[2]))

if __name__ == "__main__":
    main(*sys.argv[1:])
