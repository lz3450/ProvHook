/**
 * @name find all files in the database
 * @kind problem
 * @problem.severity recommendation
 * @id python/get-db-files
 */

import python

from File f
select f, f.getBaseName()
