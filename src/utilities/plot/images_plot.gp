# _width = 512 * 10  * 2
# _height = 512 * 10 * 4
_width=1920
_height=1080
set terminal png size _width,_height font ',32' truecolor

if (ARG1 eq ""){
	print("Error! Expecting parameter!")
	exit -1
	# map_id = "1"
} else {
	pwd_file = ARG1
}
file=system(sprintf('file=%s; file="${file##*/}";echo "${file%.txt}"', pwd_file))
print(file)

aruco_pwd="/home/pracsys/raspi_ros_perception"

_output=sprintf("%s/out/figs/%s.png", aruco_pwd, file)
print(_output)
set output _output

avg(x1,x2,x3,x4)=(x1+x2+x3+x4)/4
f(x,y,str)= (x eq str? y : 1/0)


image_1=sprintf("~/my_photo-11.jpg")
camera_1="/camera_logitech/pracsys/markers"


_ps=1
# print(txt_file)
# set multiplot layout 3,2  title "" font ",16"

plot image_1 binary filetype=jpg with rgbimage notit,\
 pwd_file u (f(strcol(1), avg($9,$11,$13,$15), camera_1)):(1080 - avg($10,$12,$14,$16)) w p ps _ps pt 5 notit
 
# plot image_2 binary filetype=jpg with rgbimage notit,\
# 	txt_file u (f(strcol(1), $9, camera_2)):(1080 - $10) w p ps _ps pt 5 notit

# plot image_3 binary filetype=jpg with rgbimage notit,\
#  txt_file u (f(strcol(1), $9, camera_3)):(720 - $10) w p ps _ps pt 5 notit

# plot image_4 binary filetype=png with rgbimage notit,\
#  txt_file u (f(strcol(1), 720- $9, camera_4)):($10) w p ps _ps pt 5 notit

# plot image_5 binary filetype=png with rgbimage notit,\
#  txt_file u (f(strcol(1), 720 - $9, camera_5)):($10) w p ps _ps pt 5 notit
