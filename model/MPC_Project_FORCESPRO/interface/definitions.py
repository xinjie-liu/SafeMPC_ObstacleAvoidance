import numpy
import ctypes

name = "MPC_Project_FORCESPRO"
requires_callback = False
lib = "lib/MPC_Project_FORCESPRO.dll"
lib_static = "lib/MPC_Project_FORCESPRO_static.lib"
c_header = "include/MPC_Project_FORCESPRO.h"
nstages = 10

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
 ("f_error1"            , "dense" , ""               , ctypes.c_double, numpy.float64, (  5,   1),    5),
 ("f_error2"            , "dense" , ""               , ctypes.c_double, numpy.float64, (  5,   1),    5),
 ("f_error3"            , "dense" , ""               , ctypes.c_double, numpy.float64, (  5,   1),    5),
 ("f_error4"            , "dense" , ""               , ctypes.c_double, numpy.float64, (  5,   1),    5),
 ("f_error5"            , "dense" , ""               , ctypes.c_double, numpy.float64, (  5,   1),    5),
 ("f_error6"            , "dense" , ""               , ctypes.c_double, numpy.float64, (  5,   1),    5),
 ("f_error7"            , "dense" , ""               , ctypes.c_double, numpy.float64, (  5,   1),    5),
 ("f_error8"            , "dense" , ""               , ctypes.c_double, numpy.float64, (  5,   1),    5),
 ("f_error9"            , "dense" , ""               , ctypes.c_double, numpy.float64, (  5,   1),    5)]

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
	(5, 3, 0, 0, 0, 0, 0, 0)
]