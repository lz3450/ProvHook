/**
 * @name get the number of parameters of the create_node function
 * @kind problem
 * @problem.severity recommendation
 * @id python/get-num-parameter
 */

import python
import semmle.python.ApiGraphs

from API::Node node, int numParams
where
  node.getLocation().getFile().getBaseName() = "myteleop.py" and
  node = API::moduleImport("rclpy").getMember("create_node") and
  numParams = node.getNumParameter()
select node, numParams.toString()
////////////////////////////////////////////////////////////////////////////////
// from API::Node node, int numParams
// where
//   node.getLocation().getFile().getBaseName() = "myteleop.py" and
//   node = API::moduleImport("rclpy").getMember("create_node").getReturn().getMember("create_publisher") and
//   numParams = node.getNumParameter()
// select node, numParams.toString()
