#!/bin/bash

if [ "$#" -ne 3 ]; then
    echo "Usage: ./generate_rqt_py_package.sh [package name] [class name] [plugin title]"
    exit
fi

directory="./"
exclude_file="./generate_rqt_py_package.sh"

package_name=$1
class_name=$2
plugin_title=$3

new_directory="../$package_name"
mkdir -p $new_directory
cp -r ./ $new_directory
cd $new_directory
rm -rf .git
rm generate_rqt_py_package.sh
cd - > /dev/null

mv $new_directory/src/rqt_py_template $new_directory/src/$package_name
mv $new_directory/resource/rqt_py_template $new_directory/resource/$package_name

for file in $(find "$new_directory" -type f); do
    sed -i "s/Template/$class_name/g" $file
    sed -i "s/rqt_py_template/$package_name/g" $file
    sed -i "s/PluginTitle/$plugin_title/g" $file
done
