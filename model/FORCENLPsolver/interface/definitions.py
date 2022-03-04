import numpy
import ctypes

name = "FORCENLPsolver"
requires_callback = True
lib = "lib/FORCENLPsolver.dll"
lib_static = "lib/FORCENLPsolver_static.lib"
c_header = "include/FORCENLPsolver.h"
nstages = 40

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   1),    3),
 ("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, (200,   1),  200),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, (120,   1),  120),
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
 ("x20"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x21"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x22"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x23"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x24"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x25"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x26"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x27"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x28"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x29"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x30"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x31"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x32"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x33"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x34"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x35"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x36"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x37"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x38"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x39"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x40"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5)]

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
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0), 
	(5, 3, 1, 3, 0, 0, 1, 0)
]