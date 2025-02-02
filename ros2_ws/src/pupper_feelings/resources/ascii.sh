#!/bin/bash

clear > "/dev/tty1"
while IFS= read -r line; do
    echo -e "$line" > "/dev/tty1"
done < $1