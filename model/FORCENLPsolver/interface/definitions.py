import numpy
import ctypes

name = "FORCENLPsolver"
requires_callback = True
lib = "lib/libFORCENLPsolver.so"
lib_static = "lib/libFORCENLPsolver.a"
c_header = "include/FORCENLPsolver.h"
nstages = 20

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   1),    3),
 ("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, (100,   1),  100),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, ( 60,   1),   60),
 ("reinitialize"        , ""      , "FORCENLPsolver_int", ctypes.c_int   , numpy.int32  , (  0,   1),    1)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x01"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x02"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x03"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x04"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x05"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x06"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x07"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x08"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x09"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x10"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x11"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x12"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x13"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x14"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x15"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x16"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x17"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x18"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x19"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x20"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5)]

# Info Struct Fields
info = \
[("it", ctypes.c_int),
 ("res_eq", ctypes.c_double),
 ("rsnorm", ctypes.c_double),
 ("pobj", ctypes.c_double),
 ("solvetime", ctypes.c_double),
 ("fevalstime", ctypes.c_double),
 ("QPtime", ctypes.c_double)]

# Dynamics dimensions
#   nvar    |   neq   |   dimh    |   dimp    |   diml    |   dimu    |   dimhl   |   dimhu    
dynamics_dims = [
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0), 
	(5, 3, 0, 3, 0, 0, 0, 0)
]