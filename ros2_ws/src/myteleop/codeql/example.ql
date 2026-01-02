/**
 * @name example
 * @kind problem
 * @problem.severity recommendation
 * @id python/example
 */

import python
import semmle.python.dataflow.new.DataFlow
import semmle.python.ApiGraphs

////////////////////////////////////////////////////////////////////////////////
from DataFlow::CallCfgNode call, DataFlow::ExprNode expr0, DataFlow::ExprNode expr1
where
  call.getLocation().getFile().getBaseName() = "myteleop.py" and
  call =
    API::moduleImport("rclpy")
        .getMember("create_node")
        .getReturn()
        .getMember("create_publisher")
        .getReturn()
        .getMember("publish")
        .getACall() and
  DataFlow::localFlow(expr0, call.getArg(0)) and
  DataFlow::localFlow(expr1, expr0.)
select call, expr1
////////////////////////////////////////////////////////////////////////////////
