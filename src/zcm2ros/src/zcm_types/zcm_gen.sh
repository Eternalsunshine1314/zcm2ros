#!/bin/bash

PATH_TO_SCR=$(echo $0 | sed 's/zcm_gen.sh//g')
cd $PATH_TO_SCR

CXX=$1

[[ -z $CXX ]] && CXX="g++"

zcm_list=$(find $(pwd) -iname '*.zcm')

gpp_str=

for zcm_file in $zcm_list
do
        file_path=$(dirname $zcm_file)

        if [ -z "${file_path}" ]; then
                echo "file_path is empty string"
                exit 0
        fi

        rm -rf $file_path/c_types/
        rm -rf $file_path/cpp_types/
        rm -rf $file_path/py_types/

        gpp_str=${gpp_str}" -I ${file_path}/c_types/ ${file_path}/c_types/*.c"


        zcm-gen --lazy -c --c-typeinfo  --c-cpath $file_path/c_types/ --c-hpath $file_path/c_types/ $zcm_file
        zcm-gen --lazy -x --cpp-hpath $file_path/cpp_types/ $zcm_file
        zcm-gen --lazy -p --ppath $file_path/py_types/ $zcm_file
        touch $file_path/py_types/__init__.py

done

$CXX -c -fpic $CPPFLAGS $gpp_str || exit 1
$CXX -shared -o libzcmtypes.so *.o || exit 1
rm *.o
