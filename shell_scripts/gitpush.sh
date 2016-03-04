#!/bin/bash

git pull
branch=$(git name-rev --name-only HEAD)
git add --all
echo $(pwd)
echo git commit -am \"$1\"
echo git push origin $branch
