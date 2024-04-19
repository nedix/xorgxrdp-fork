#!/bin/sh

curl https://raw.githubusercontent.com/wiki/neutrinolabs/xorgxrdp/NEWS-v0.10.md > $(git rev-parse --show-toplevel)/NEWS.md
