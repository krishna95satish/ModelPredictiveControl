# vehicle control parameters
# set parameter for PID in lateral direction
float32 qrg_v # Proportional term based on error of heading angle
float32 qrg_bfryt # Proportional term based on error of distance
float32 qrg_bfrydt # Differential term based on error of distance
float32 qrg_bfryit # Integral term based on error of distance
float32 qrg_ff # feedforward gain
float32 qrg_sgs # set parameter for disturbance observer
# set parameter for PID in longitudinal direction
float32 lrg_bfrt # Proportional term based on error of speed
float32 lrg_bfrdt # Differential term based on error of speed
float32 lrg_bfrit # Integral term based on error of speed
float32 lrg_bfrxt # Proportional term based on error of distance
float32 lrg_bfrxdt # Differential term based on error of distance
float32 lrg_bfrxit # Integral term based on error of distance