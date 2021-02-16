import matplotlib.pyplot as plt
import os

def is_float(string):
    try:
        float(string)
        return True
    except ValueError:
        return False

def read_file(filename):
    f = open(filename, 'r')
    result = [float(n) for n in f.read().split('\n') if is_float(n)]
    return result

def make_graph(filename):
    if os.path.isfile(filename):
        y = read_file(filename)
        x = list(range(len(y)))
        zero_legend_x = list(range(-100, len(y) + 101))
        zero_legend_y = [0] * len(zero_legend_x)
        plt.plot(x,y)
        plt.plot(zero_legend_x, zero_legend_y)
        plt.xlim(left=0)
        plt.ylim(top=4, bottom=-4)
        plt.show()
    else:
        print('Error: no such file %s.' % filename)
        return

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print('Usage: %s <filename>' % sys.argv[0])
        sys.exit(0)
    else:
        make_graph(sys.argv[1])
