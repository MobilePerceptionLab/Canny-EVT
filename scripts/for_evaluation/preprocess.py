import glob

text_files = glob.glob("*.txt")

for path in text_files:
    fout = open("./ok2compare/"+path, "w")
    fin = open(path, "r")
    init_ts = 0.0
    for (i, line) in enumerate(fin.readlines()):
        els = [s.strip() for s in line.split(" ") if s != ""]
        if (i == 0):
            init_ts = float(els[0])
        els[0] = "{:9.6f}".format(float(els[0])-init_ts)
        for e in els[0:-1]:
            print(e.strip(), end=" ", file=fout)
        print(els[-1].strip(), file=fout)

    fin.close()
    fout.close()

text_files = glob.glob("*.csv")
n_head = 7

for path in text_files:
    fin = open(path, "r")
    path = path.replace(".csv", ".txt")
    fout = open("./ok2compare/"+path, "w")
    init_ts = 0.0
    for (i, line) in enumerate(fin.readlines()):
        if i < n_head:
            continue
        # print(els)
        line=line.strip()
        els = [float(s.strip()) for s in line.split(",") if s != ""]

        if (i == n_head):
            init_ts = els[1]

        to_write = "{:9.6f} {:f} {:f} {:f} {:f} {:f} {:f} {:f}".format(
            els[1], els[6], els[7], els[8], els[2], els[3], els[4], els[5])
        print(to_write.strip(),  file=fout)

    fin.close()
    fout.close()
