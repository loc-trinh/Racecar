#!/bin/bash

branch=$(git name-rev --name-only HEAD)
direc=$(pwd)
echo "Operating on branch $branch of git repo at: $direc"
echo "pulling..."
echo $(git pull)
echo "committing..."
echo $(git add --all)
echo git commit -am \"$1\"
echo $(git commit -am \"$1\")
echo "pushing..."
echo git push origin $branch
