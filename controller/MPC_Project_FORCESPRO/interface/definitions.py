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
[("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  6,   1),    6),
 ("linear_model1"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  6,  10),   60),
 ("linear_model2"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  6,  10),   60),
 ("linear_model3"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  6,  10),   60),
 ("linear_model4"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  6,  10),   60),
 ("linear_model5"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  6,  10),   60),
 ("linear_model6"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  6,  10),   60),
 ("linear_model7"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  6,  10),   60),
 ("linear_model8"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  6,  10),   60),
 ("linear_model9"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  6,  10),   60),
 ("hyperplaneA1"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,  10),   10),
 ("hyperplaneA2"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,  10),   10),
 ("hyperplaneA3"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,  10),   10),
 ("hyperplaneA4"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,  10),   10),
 ("hyperplaneA5"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,  10),   10),
 ("hyperplaneA6"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,  10),   10),
 ("hyperplaneA7"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,  10),   10),
 ("hyperplaneA8"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,  10),   10),
 ("hyperplaneA9"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,  10),   10),
 ("hyperplaneA10"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,  10),   10),
 ("hyperplaneb1"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,   1),    1),
 ("hyperplaneb2"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,   1),    1),
 ("hyperplaneb3"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,   1),    1),
 ("hyperplaneb4"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,   1),    1),
 ("hyperplaneb5"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,   1),    1),
 ("hyperplaneb6"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,   1),    1),
 ("hyperplaneb7"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,   1),    1),
 ("hyperplaneb8"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,   1),    1),
 ("hyperplaneb9"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,   1),    1),
 ("hyperplaneb10"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  1,   1),    1)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("output"              , ""      , ""               , ctypes.c_double, numpy.float64,     ( 10,),  100)]

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
	(10, 6, 0, 0, 0, 0, 0, 0), 
	(10, 6, 0, 0, 0, 0, 0, 0), 
	(10, 6, 0, 0, 0, 0, 0, 0), 
	(10, 6, 0, 0, 0, 0, 0, 0), 
	(10, 6, 0, 0, 0, 0, 0, 0), 
	(10, 6, 0, 0, 0, 0, 0, 0), 
	(10, 6, 0, 0, 0, 0, 0, 0), 
	(10, 6, 0, 0, 0, 0, 0, 0), 
	(10, 6, 0, 0, 0, 0, 0, 0), 
	(10, 6, 0, 0, 0, 0, 0, 0)
]