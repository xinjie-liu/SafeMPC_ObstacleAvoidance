import numpy
import ctypes

name = "MPC_Project_FORCESPRO"
requires_callback = False
lib = "lib/MPC_Project_FORCESPRO.dll"
lib_static = "lib/MPC_Project_FORCESPRO_static.lib"
c_header = "include/MPC_Project_FORCESPRO.h"
nstages = 50

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   1),    3),
 ("linear_model1"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model2"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model3"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model4"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model5"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model6"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model7"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model8"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model9"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model10"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model11"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model12"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model13"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model14"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model15"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model16"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model17"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model18"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model19"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model20"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model21"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model22"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model23"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model24"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model25"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model26"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model27"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model28"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model29"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model30"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model31"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model32"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model33"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model34"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model35"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model36"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model37"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model38"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model39"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model40"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model41"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model42"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model43"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model44"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model45"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model46"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model47"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model48"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15),
 ("linear_model49"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   5),   15)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("output"              , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),   50)]

# Info Struct Fields
info = \
[('it', ctypes.c_int32),
('it2opt', ctypes.c_int32),
('res_eq', ctypes.c_double),
('res_ineq', ctypes.c_double),
('pobj', ctypes.c_double),
('dobj', ctypes.c_double),
('dgap', ctypes.c_double),
('rdgap', ctypes.c_double),
('gradient_lag_norm', ctypes.c_double),
('mu', ctypes.c_double),
('mu_aff', ctypes.c_double),
('sigma', ctypes.c_double),
('lsit_aff', ctypes.c_int32),
('lsit_cc', ctypes.c_int32),
('step_aff', ctypes.c_double),
('step_cc', ctypes.c_double),
('solvetime', ctypes.c_double)
]

# Dynamics dimensions
#   nvar    |   neq   |   dimh    |   dimp    |   diml    |   dimu    |   dimhl   |   dimhu    
dynamics_dims = [
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0), 
	(5, 3, 0, 0, 0, 0, 0, 0)
]