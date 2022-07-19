#!/usr/bin/env python
# ============================== GENERAL LOS =========================================
def sign(v):
    if v > 0:
        return 1
    elif v < 0:
        return -1
    else:
        return 0
#l = 0
#s = 0
#i = 0 
#j = 0
#wl = 0
#ws = 0
#ds = 0
#dl = 0
#p = 0
abs_Dl = 0
#n = 0
#e = 0
#LL = 0
#gi = gi0
#hbe = hbe0
#wait = False

def gi0(l, s):
    return (l, s)
def gi1(l, s):
    return (s, l)
def gi2(l, s):
    global i, j
    i = l
    j = s
def gi3(l, s):
    global i, j
    i = s
    j = l
def hbe0(e):
    return e >= abs_Dl
def hbe1(e):
    return e < -abs_Dl
    
def get_los_path(si, sj, k, ei, ej, ek, num_j):
    global abs_Dl
    # general int los implementation
    path = []
    # start idx and end_idx must  be int
    Di = ei - si
    Dj = ej - sj
    if abs(Di) > abs(Dj):
        Dl = Di
        Ds = Dj
        ds = sign(Ds)
        dkl = num_j
        if Dl <= 0:
            dkl = -num_j
        dks = ds
    else:
        Dl = Dj
        Ds = Di
        ds = sign(Ds)
        dkl = sign(Dl)
        dks = num_j
        if ds <= 0:
            dks = -num_j
    p = 2*Ds
    abs_Dl = abs(Dl)
    n = 2*abs_Dl*ds
    e = 0
    LL = abs(Ds) - abs_Dl
    if Ds >= 0:
        hbe = hbe0
    else:
        hbe = hbe1
    while k != ek:
        k += dkl
        e += p
        if hbe(e):
            e -= n
            k += dks
            ll = e * ds
            if ll < LL:
                path.append(k-dks)
            elif ll > LL:
                path.append(k-dkl)
        path.append(k)
    return path
    


#class GeneralIntLOS:
#    def __init__(self):
#        self.i = 0; self.j = 0
#        self.wl = 0; self.wr = 0;
#    def init(self, si, sj, ei, ej):
#        # start idx and end_idx must  be int
#        Di = ei - si
#        Dj = ej - sj
#        if abs(Di) > abs(Dj):
#            Dl = Di
#            Ds = Dj
#            self.gi = self.gi0
#            self.l = si
#            self.s = sj
#        else:
#            Dl = Dj
#            Ds = Di
#            self.gi = self.gi1
#            self.l = sj
#            self.s = si
#        self.ds = sign(Ds)
#        self.dl = sign(Dl)
#        self.p = 2*Ds
#        t = abs(Dl)
#        self.n = 2*t*self.ds
#        self.e = 0
#        self.LL = abs(Ds) - t
#        if Ds >= 0:
#            self.hbe = lambda e : e >= t
#        else:
#            self.hbe = lambda e : e < -t
#        self.wait = False
#        self.i = si; self.j = sj
#    def next(self):
#        if self.wait:
#            self.gi(self.wl, self.ws)
#            self.wait = False
#            return
#        self.l += self.dl
#        self.e += self.p
#        if self.hbe(self.e):
#            self.e -= self.n
#            self.s += self.ds
#            ll = self.e * self.ds
#            if ll < self.LL:
#                self.gi(self.l, self.s-self.ds)
#            elif ll > self.LL:
#                self.gi(self.l-self.dl, self.s)
#            self.wl = self.l; self.ws = self.s
#            self.wait = True
#            return
#        self.gi(self.l, self.s)
#    def gi0(self, l, s):
#        self.i = l; self.j = s
#    def gi1(self, l, s):
#        self.i = s; self.j = l
#class GeneralLOS:
#    def __init__(self):
#        self.i = 0; self.j = 0
#        self.wl = 0; self.wr = 0;
#    def init(self, si, sj, ei, ej):
#        Di = ei - si
#        Dj = ej - sj
#        if abs(Di) > abs(Dj):
#            Dl = Di
#            Ds = Dj
#            self.gi = self.gi0
#            self.l = int(round(si)); self.s = int(round(sj))
#            self.i = self.l; self.j = self.s
#            L = si; S = sj
#        else:
#            Dl = Dj
#            Ds = Di
#            self.gi = self.gi1
#            self.l = int(round(sj)); self.s = int(round(si))
#            self.i = self.s; self.j = self.l
#            L = sj; S = si
#        self.ds = int(sign(Ds))
#        self.dl = int(sign(Dl))
#        self.ps = Ds / abs(Dl)
#        self.e_s = S - self.s
#        self.L = abs(Ds / Dl) * (0.5 + (L - self.l)*self.dl) - 0.5
#        if Ds >= 0:
#            self.hbe = lambda e_s : e_s >= 0.5
#        else:
#            self.hbe = lambda e_s : e_s < -0.5
#        self.wait = False
#    def next(self):
#        if self.wait:
#            self.gi(self.wl, self.ws)
#            self.wait = False
#            return
#            
#        self.l += self.dl
#        self.e_s += self.ps
#        if self.hbe(self.e_s):
#            self.e_s -= self.ds
#            self.s += self.ds
#            L = self.e_s * self.ds
#            if L < self.L:
#                self.gi(self.l, self.s-self.ds)
#            elif L > self.L:
#                self.gi(self.l-self.dl, self.s)
#            #else:
#                # self.list.append((self.l, self.s))
#                # self.list.append((self.l, self.s-self.ds))
#                # return self.to_int(self.gi((self.l-self.dl, self.s)))
#            self.wl = self.l; self.ws = self.s
#            self.wait = True
#            return
#        self.gi(self.l, self.s)
#            
#    def gi0(self, l, s):
#        self.i = l
#        self.j = s
#    def gi1(self, l, s):
#        self.i = s
#        self.j = l
