import numpy as np;
import libs.mpyc as reg;

def initialize_mpc():
    # System matrices
    A = np.matrix([[0.968022]]);
    B = np.matrix([[0.000154882,0.00332278,0.000285817]]);
    C = np.matrix([[55811.1],[0.315664]]);
    D = np.matrix([[0,0,0],[0,0,0]]);
    #print "a"
    #print A
    #print "b"
    #print B
    #print "c"
    #print C
    #print "d"
    #print D
    # Control parameters
    L = 4;
    Q = np.diag(np.tile(np.matrix([[100,0.001]]),[1,L]).tolist()[0]);
    R = np.diag(np.tile(np.matrix([[100,100000,100000]]),[1,L]).tolist()[0]);
    #print "q"
    #print Q
    #print "r"
    #print R

    # Control saturations
    Umin = np.matrix([[1],[0],[0]]);
    Umax = np.matrix([[99],[5],[5]]);
    DeltaUmin = np.matrix([[-30],[-3],[-3]]);
    DeltaUmax = np.matrix([[30],[3],[3]]);

    #print "umin"
    #print Umin
    #print "umax"
    #print Umax
    #print "deltaumin"
    #print DeltaUmin
    #print "deltaumax"
    #print DeltaUmax

    # Kalman filter matrices
    Qn = np.diag(np.matrix([[1,1,1]]).tolist()[0]);
    Rn = np.diag(np.matrix([[1,1]]).tolist()[0]);
    Lk = np.matrix([[1.73441e-05,9.80971e-11]]);
    Pk = np.matrix([[1.11468e-05]]);

    #print "qn"
    #print Qn
    #print "rn"
    #print Rn
    #print "lk"
    #print Lk
    #print "pk"
    #print Pk
    # Setpoint
    sp = np.matrix([[0.9],[25000]]);
    #print "sp"
    #print sp
    # Initializing the controller
    controller = reg.MPCController(A, B, C, D, L, Q, R, Lk, Pk, Qn, Rn,	Umin, Umax, DeltaUmin, DeltaUmax,	optim = 1, fast = 0, time_varying = 1)
    #print "controller"
    #print controller
    return controller;

