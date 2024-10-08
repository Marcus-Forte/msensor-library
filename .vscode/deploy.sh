#/bin/bash

rsync -avu --exclude 'include/' --exclude 'lib/' $1 $2