#!/bin/bash

rm codeql/db/db-python/default/cache/.lock
codeql query run --database=codeql/db codeql/example.ql
