#/bin/bash

rsync -avu --exclude 'include/' --exclude 'lib/' $@