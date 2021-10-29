#https://github.com/MichaelGrupp/evo/wiki/Plotting

evo_config reset

#Set the plot grid and backround
evo_config set plot_seaborn_style whitegrid #Suitable for paper prints

#Set font type and scale
evo_config set plot_fontfamily serif plot_fontscale 1.2
evo_config set plot_linewidth 1.0
evo_config set plot_reference_linestyle -


#Set the default figure size
evo_config set plot_figsize 5 4.5

#Coordinate axis markers
#...

#Plot the error mapped onto trajectory
#...

#Plot 2D ROS maps
#...

#Set configuratoins into a .json file instead
#https://github.com/MichaelGrupp/evo/wiki/Configuration

#Save result zip files for evo_res
#https://github.com/MichaelGrupp/evo/wiki/Formats
#evo_config set save_traj_in_zip true



evo_config set table_export_format latex
evo_config set plot_usetex