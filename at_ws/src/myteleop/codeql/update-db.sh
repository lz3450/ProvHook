#!/bin/bash

rm -rf db
codeql database create db --language=python --source-root="../"
