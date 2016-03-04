#!/bin/bash

git pull
branch=$(git branch)
git add --all
git commit -am "$1"
git push origin $branch
