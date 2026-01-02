/**
 * @name get the expression of dataflow node that flows to the message argument of the publish function
 * @kind problem
 * @problem.severity recommendation
 * @id python/get-dataflow-expr
 */

 import python
 import semmle.python.dataflow.new.DataFlow
 import semmle.python.ApiGraphs

 from DataFlow::CallCfgNode call, DataFlow::ExprNode expr
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
   DataFlow::localFlow(expr, call.getArg(0))
 select call, expr.asExpr().
