import matplotlib.pyplot as plt

filename = "points2.txt"   # your CSV-like file

ns, xs, ys = [], [], []

with open(filename, "r") as f:
    for line in f:
        parts = line.strip().split(",")
        if len(parts) == 3:
            n = int(parts[0])
            x = float(parts[1])
            y = float(parts[2])

            ns.append(n)
            xs.append(x)
            ys.append(y)

# Plot X over time n
plt.figure(figsize=(10,5))
plt.plot(ns, xs, label="x(n)", color="blue")
plt.xlabel("n")
plt.ylabel("x")
plt.title("X over time")
plt.grid(True)
plt.legend()
plt.show()

# Plot Y over time n
plt.figure(figsize=(10,5))
plt.plot(ns, ys, label="y(n)", color="red")
plt.xlabel("n")
plt.ylabel("y")
plt.title("Y over time")
plt.grid(True)
plt.legend()
plt.show()
