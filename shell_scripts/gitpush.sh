#!/bin/bash

branch=$(git name-rev --name-only HEAD)
direc=$(pwd)
echo "Operating on branch $branch of git repo at: $direc"
echo "==== PULLING ===="
echo $(git pull)
echo "==== COMMITTING ===="
echo $(git add --all)
echo $(git commit -a -m "$1")
echo "==== PUSHING ===="
echo $(git push origin $branch)
