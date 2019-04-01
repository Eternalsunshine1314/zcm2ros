#!/bin/bash

PATH_TO_SCR=$(echo $0 | sed 's/doxy_gen.sh//g')
cd $PATH_TO_SCR

zcm_list=$(find $(pwd) -iname '*.zcm')

for zcm_file in $zcm_list
do
	# echo $zcm_file
	sed -i 's/.*}.*/};/' $zcm_file

	file_path=$(dirname $zcm_file)
	echo $file_path

	input_str_num=$(sed -n '/INPUT                  =/=' Doxyfile.conf)
	echo $input_str_num 

	if [ ! -z $input_str_num ]; then

		sed -i "${input_str_num}c\INPUT                  = ${file_path}" Doxyfile.conf
	else 
		echo "wrong doxygen config file"
		exit 0

	fi

	output_str_num=$(sed -n '/OUTPUT_DIRECTORY       =/=' Doxyfile.conf)
	echo $output_str_num 

	if [ ! -z $output_str_num ]; then

		sed -i "${output_str_num}c\OUTPUT_DIRECTORY       =  ${file_path}/doc" Doxyfile.conf
	else 
		echo "wrong doxygen config file"
		exit 0

	fi

	doxygen Doxyfile.conf

	sed -i 's/.*};*.*/}/' $zcm_file

	sed -i "${input_str_num}c\INPUT                  = " Doxyfile.conf
	sed -i "${output_str_num}c\OUTPUT_DIRECTORY       =  " Doxyfile.conf

done	

exit 0
