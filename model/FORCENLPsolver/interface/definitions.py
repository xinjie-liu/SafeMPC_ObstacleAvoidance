import numpy
import ctypes

name = "FORCENLPsolver"
requires_callback = True
lib = "lib/FORCENLPsolver.dll"
lib_static = "lib/FORCENLPsolver_static.lib"
c_header = "include/FORCENLPsolver.h"
nstages = 30

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  6,   1),    6),
 ("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, (330,   1),  330),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, (180,   1),  180),
 ("reinitialize"        , ""      , "FORCENLPsolver_int", ctypes.c_int   , numpy.int32  , (  0,   1),    1)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x01"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x02"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x03"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x04"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x05"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x06"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x07"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x08"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x09"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x10"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x11"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x12"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x13"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x14"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x15"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x16"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x17"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x18"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x19"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x20"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x21"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x22"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x23"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x24"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x25"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x26"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x27"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x28"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x29"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x30"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11)]

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
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0), 
	(11, 6, 2, 6, 0, 0, 2, 0)
]