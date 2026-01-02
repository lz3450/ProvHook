/**
 * @name get ROS2 node publisher topic name
 * @kind problem
 * @problem.severity recommendation
 * @id python/get-topic-name
 */

import python
import semmle.python.dataflow.new.DataFlow
import semmle.python.ApiGraphs

from DataFlow::CallCfgNode call, Expr arg, string value
where
  call.getLocation().getFile().getBaseName() = "myteleop.py" and
  call =
    API::moduleImport("rclpy")
        .getMember("create_node")
        .getReturn()
        .getMember("create_publisher")
        .getACall() and
  arg = call.getArg(1).asExpr() and
  arg instanceof StringLiteral and
  value = arg.(StringLiteral).getLiteralValue().toString()
select call.getArg(1), value
