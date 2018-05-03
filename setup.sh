#!/bin/sh

echo "Deploying git hooks"

HOOKDIR=.git/hooks

# remove old hooks
rm $HOOKDIR/*

# copy new hooks, make executable
cp hooks/* $HOOKDIR/
for file in .git/hooks/*
do
  chmod u+x $file
done

