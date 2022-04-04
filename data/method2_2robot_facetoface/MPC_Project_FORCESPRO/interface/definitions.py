import numpy
import ctypes

name = "MPC_Project_FORCESPRO"
requires_callback = False
lib = "lib/libMPC_Project_FORCESPRO.so"
lib_static = "lib/libMPC_Project_FORCESPRO.a"
c_header = "include/MPC_Project_FORCESPRO.h"
nstages = 3

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  6,   1),    6),
 ("terminal_cost"       , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,  10),  100),
 ("linear_model1"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  6,  10),   60),
 ("linear_model2"       , "dense" , ""               , ctypes.c_double, numpy.float64, (  6,  10),   60),
 ("hyperplaneA1"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,  10),   20),
 ("hyperplaneA2"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,  10),   20),
 ("hyperplaneA3"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,  10),   20),
 ("hyperplaneb1"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2),
 ("hyperplaneb2"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2),
 ("hyperplaneb3"        , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("output"              , ""      , ""               , ctypes.c_double, numpy.float64,     ( 10,),   30)]

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
	(10, 6, 0, 0, 2, 2, 0, 0), 
	(10, 6, 0, 0, 2, 2, 0, 0), 
	(10, 6, 0, 0, 2, 2, 0, 0)
]