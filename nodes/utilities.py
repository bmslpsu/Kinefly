
import os
import numpy as np
import pickle, json
import Tkinter
import tkFileDialog as filedialog

def stepArray(start, step, stop):
    a = np.arange(start, stop + step, step)
    return np.asarray(a)

def combvec(*args):
    y = args[0]
    n_vec = len(args)
    for i in stepArray(2, 1, n_vec):
        z = args[i-1]
        if z.ndim == 2:
            nz = z.shape[1]
        else:
            nz = len(z)

        if y.ndim == 2:
            ny= y.shape[1]
        else:
            ny = len(y)

        a = copy_blocked(y,nz)
        b = copy_interleaved(z,ny)
        y = np.concatenate((a,b), axis=0)

    return y.T

def copy_blocked(m,n):
    if m.ndim == 2:
        mr = m.shape[0]
        mc = m.shape[1]
    else:
        mr = 1
        mc = len(m)

    b = np.zeros((mr, mc*n))
    ind = stepArray(1,1,mc)
    v = stepArray(0,1,n-1)*mc
    for i in v:
        b[:,ind+i-1] = m

    return b

def copy_interleaved(m, n):
    if m.ndim == 2:
        mr = m.shape[0]
        mc = m.shape[1]
    else:
        mr = 1
        mc = len(m)

    b = np.zeros((mr*n, mc))
    ind = stepArray(1,1,mr)
    v = stepArray(0,1,n-1)*mr
    for i in v:
        b[ind+i-1,:] = m

    # b = b.reshape((mr, mc*n), order='F')

    # output = np.transpose(b)
    # output = output.reshape((mr, mc*n), order='F')
    output = b.reshape((mr, mc*n), order='F')

    return output


def loadpickle(root=None):

    if root is None:
        root = ''

    fileparts = os.path.splitext(root) # split path

    if not fileparts[-1] == '.pickle': # given a file path to .pickle
        window = Tkinter.Tk()  # initalize tkinter
        window.withdraw()  # close unecessary window

        # Open file selection GUI
        files = filedialog.askopenfilenames(initialdir=root, title="Select start file", filetypes=(
            ("all files", "*.*"),
            ("mat files", '*.mat')))
        filepath = files[0]

    else: # given a directory
        filepath = root

    # Load .pickle file
    with open(filepath, 'rb') as f:
        data = pickle.load(f)

    return data

def loadjson(root=None):

    if root is None:
        root = ''

    fileparts = os.path.splitext(root) # split path

    if not fileparts[-1] == '.json': # given a file path to .pickle
        window = Tkinter.Tk()  # initalize tkinter
        window.withdraw()  # close unecessary window

        # Open file selection GUI
        files = filedialog.askopenfilenames(initialdir=root, title="Select start file", filetypes=(
            ("all files", "*.*"),
            ("mat files", '*.mat')))
        filepath = files[0]

    else: # given a directory
        filepath = root

    # Load .pickle file
    with open(filepath, 'rb') as f:
        data = json.load(f)

    return data

def getDictFields(dict, fields):
    out = { }
    # keys = dict.keys()
    fields.sort()
    for f in range(len(fields)):
        # print(fields[f])
        data = dict[fields[f]]
        if isinstance(data, (list, np.ndarray, np.matrix)):
            data = data.tolist()
        out[fields[f]] = data

    return out


if __name__ == '__main__':
    k1 = stepArray(0,5,200)
    k2 = stepArray(0,5,50)
    # k3 = stepArray(21,1,30)
    k3 = np.asarray([0])
    K = combvec(k1,k2,k3)
    print('here')
    print('done')