import ast

# Read the content of the uploaded Python file
file_path = '../myteleop.py'
with open(file_path, 'r') as file:
    code = file.read()

# Parse the Python code into an Abstract Syntax Tree (AST)
parsed_code = ast.parse(code)

class DataFlowVisitor(ast.NodeVisitor):
    def __init__(self):
        self.edges = []
        self.variables = set()

    def visit_Assign(self, node):
        targets = [self.get_source(target) for target in node.targets]
        value = self.get_source(node.value)
        for target in targets:
            self.edges.append((value, target))
            self.variables.add(target)
        self.generic_visit(node)

    def visit_AugAssign(self, node):
        target = self.get_source(node.target)
        value = self.get_source(node.value)
        op = self.get_op(node.op)
        self.edges.append((f"{target} {op} {value}", target))
        self.variables.add(target)
        self.generic_visit(node)

    def visit_Call(self, node):
        func_name = self.get_source(node.func)
        for arg in node.args:
            self.edges.append((self.get_source(arg), func_name))
        self.generic_visit(node)

    def visit_FunctionDef(self, node):
        self.generic_visit(node)

    def visit_Return(self, node):
        if node.value:
            self.edges.append((self.get_source(node.value), 'return'))
        self.generic_visit(node)

    def get_source(self, node):
        if isinstance(node, ast.Name):
            return node.id
        elif isinstance(node, ast.Constant):
            return str(node.value)
        elif isinstance(node, ast.BinOp):
            return f'({self.get_source(node.left)} {self.get_op(node.op)} {self.get_source(node.right)})'
        elif isinstance(node, ast.Call):
            func_name = self.get_source(node.func)
            args = ', '.join(self.get_source(arg) for arg in node.args)
            return f'{func_name}({args})'
        elif isinstance(node, ast.Attribute):
            return f'{self.get_source(node.value)}.{node.attr}'
        elif isinstance(node, ast.Subscript):
            return f'{self.get_source(node.value)}[{self.get_source(node.slice)}]'
        elif isinstance(node, ast.Index):
            return self.get_source(node.value)
        return 'unknown'

    def get_op(self, op):
        if isinstance(op, ast.Add):
            return '+'
        elif isinstance(op, ast.Sub):
            return '-'
        elif isinstance(op, ast.Mult):
            return '*'
        elif isinstance(op, ast.Div):
            return '/'
        elif isinstance(op, ast.Mod):
            return '%'
        elif isinstance(op, ast.Pow):
            return '**'
        elif isinstance(op, ast.LShift):
            return '<<'
        elif isinstance(op, ast.RShift):
            return '>>'
        elif isinstance(op, ast.BitOr):
            return '|'
        elif isinstance(op, ast.BitXor):
            return '^'
        elif isinstance(op, ast.BitAnd):
            return '&'
        elif isinstance(op, ast.FloorDiv):
            return '//'
        return 'op'

visitor = DataFlowVisitor()
visitor.visit(parsed_code)

# Create the .dot file content
dot_lines = ["digraph G {"]
for edge in visitor.edges:
    dot_lines.append(f'    "{edge[0]}" -> "{edge[1]}";')
dot_lines.append("}")

dot_content = "\n".join(dot_lines)

# Save the .dot file
dot_file_path = 'data_flow_graph.dot'
with open(dot_file_path, 'w') as dot_file:
    dot_file.write(dot_content)
