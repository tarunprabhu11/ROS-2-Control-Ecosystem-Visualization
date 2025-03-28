#!/usr/bin/env python3

import os
import math
from threading import Lock
import time

import rclpy
from rclpy.node import Node

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, QPointF, QRectF
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QGraphicsView, QGraphicsScene, 
    QGraphicsItem, QGraphicsTextItem, QGraphicsEllipseItem, QGraphicsLineItem,
    QGraphicsPolygonItem, QGraphicsRectItem, QScrollArea, QHBoxLayout,
    QTableWidget, QTableWidgetItem, QHeaderView
)
from python_qt_binding.QtGui import QPen, QBrush, QColor, QFont, QPolygonF, QPainter

from custom_msgs.msg import NodeConnection

class SafeLogger:
    # for error messages
    @staticmethod
    def log_error(message):
        print(f"Error: {message}")

class NodeGraphNode(Node):
    # ROS node for handling node connection subscriptions
    def __init__(self, callback):
        super().__init__('rqt_node_graph')
        self.callback = callback
        self._subscriptions = []
        self.last_message_times = {}
        
        self._subscriptions.append(self.create_subscription(
            NodeConnection, 'a_to_b', lambda msg: self.listener_callback(msg, 'a_to_b'), 10))
        self._subscriptions.append(self.create_subscription(
            NodeConnection, 'b_to_c', lambda msg: self.listener_callback(msg, 'b_to_c'), 10))
        self._subscriptions.append(self.create_subscription(
            NodeConnection, 'c_to_d', lambda msg: self.listener_callback(msg, 'c_to_d'), 10))
    
    def listener_callback(self, msg, topic):
        self.last_message_times[msg.start] = time.time()
        self.callback(msg, topic)

class GraphNode:
    # Node Properties
    def __init__(self, name, is_hardware=False):
        self.name = name
        self.is_hardware = is_hardware
        self.x = 0
        self.y = 0
        self.last_active_time = time.time()
        self.is_active = False
        self.connections = []

class GraphNodeView(QGraphicsEllipseItem):
    # Visual graph node
    def __init__(self, node, scene, parent=None):
        super().__init__(0, 0, 60, 60, parent)
        self.node = node
        self.scene = scene
        
        if node.is_hardware:
            self.setBrush(QBrush(QColor('green')))
        else:
            self.setBrush(QBrush(QColor('lightgray')))
        
        self.setPen(QPen(Qt.black, 2))
        self.setFlag(QGraphicsItem.ItemIsMovable)
        
        self.text = QGraphicsTextItem(node.name, self)
        self.text.setFont(QFont("Arial", 12, QFont.Bold))
        text_rect = self.text.boundingRect()
        ellipse_rect = self.boundingRect()
        self.text.setPos(
            ellipse_rect.center().x() - text_rect.width() / 2,
            ellipse_rect.center().y() - text_rect.height() / 2
        )
    
    def get_connection_point(self, target_pos):
        center = self.sceneBoundingRect().center()
        dx = target_pos.x() - center.x()
        dy = target_pos.y() - center.y()
        
        angle = math.atan2(dy, dx)
        
        rx = self.rect().width() / 2
        ry = self.rect().height() / 2
        x = center.x() + rx * math.cos(angle)
        y = center.y() + ry * math.sin(angle)
        
        return QPointF(x, y)

class GraphConnectionView(QGraphicsLineItem):
    # Visual connection between nodes
    def __init__(self, source_node_view, target_node_view, connection_type, metadata='', timestamp=None, connection_data=None, parent=None):
        start_rect = source_node_view.sceneBoundingRect()
        end_rect = target_node_view.sceneBoundingRect()
        
        start_point = source_node_view.get_connection_point(end_rect.center())
        end_point = target_node_view.get_connection_point(start_rect.center())
        
        super().__init__(start_point.x(), start_point.y(), end_point.x(), end_point.y(), parent)
        
        self.source_node_view = source_node_view
        self.target_node_view = target_node_view
        self.connection_type = str(connection_type)
        self.metadata = str(metadata)
        self.timestamp = timestamp
        self.connection_data = connection_data
        
        color_map = {
            'command': QColor('red'),
            'state': QColor('blue')
        }
        self.default_color = color_map.get(connection_type, QColor('gray'))
        self.highlight_color = QColor('yellow')
        
        self.arrow_head = QGraphicsPolygonItem(self)
        self.update_arrow()
        
        self.update_connection_style(self.default_color)
        
        self.setFlag(QGraphicsItem.ItemIsSelectable)
    
    def update_arrow(self):
        end = self.line().p2()
        start = self.line().p1()
        
        dx = end.x() - start.x()
        dy = end.y() - start.y()
        angle = math.atan2(dy, dx)
        
        arrow_size = 10
        
        arrow_p1 = QPointF(
            end.x() - arrow_size * math.cos(angle - math.pi/6),
            end.y() - arrow_size * math.sin(angle - math.pi/6)
        )
        arrow_p2 = QPointF(
            end.x() - arrow_size * math.cos(angle + math.pi/6),
            end.y() - arrow_size * math.sin(angle + math.pi/6)
        )
        
        polygon = QPolygonF([end, arrow_p1, arrow_p2])
        self.arrow_head.setPolygon(polygon)
    
    def update_connection_style(self, color):
        pen = QPen(color, 2)
        pen.setCapStyle(Qt.RoundCap)
        self.setPen(pen)
        
        brush = QBrush(color)
        self.arrow_head.setBrush(brush)
        self.arrow_head.setPen(QPen(color, 2))
    
    def mousePressEvent(self, event):
        if self.pen().color() == self.default_color:
            self.update_connection_style(self.highlight_color)
            if hasattr(self.scene(), 'parent_widget'):
                self.scene().parent_widget.highlight_connection_row(self.connection_data)
        else:
            self.update_connection_style(self.default_color)
            if hasattr(self.scene(), 'parent_widget'):
                self.scene().parent_widget.reset_table_highlight()
        
        super().mousePressEvent(event)
    
    def mouseReleaseEvent(self, event):
        self.update_arrow()
        super().mouseReleaseEvent(event)

class NodeGraphWidget(QWidget):
    # Main widget 
    def __init__(self, parent=None):
        super(NodeGraphWidget, self).__init__(parent)
        
        self.node_inactivity_threshold = 5.0
        
        self.nodes = {}
        self.connections = {}
        self.node_views = {}
        self.connection_views = []
        self.mutex = Lock()
        
        self.main_layout = QHBoxLayout()
        self.setLayout(self.main_layout)
        
        self.graph_panel = QWidget()
        self.graph_layout = QVBoxLayout()
        self.graph_panel.setLayout(self.graph_layout)
        
        self.scene = QGraphicsScene(self)
        self.view = QGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.Antialiasing)
        self.view.setDragMode(QGraphicsView.ScrollHandDrag)
        self.view.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.graph_layout.addWidget(self.view)
        
        self.legend_layout = QHBoxLayout()
        
        self.command_label = QLabel("Command Interface")
        self.command_color = QLabel("   ")
        self.command_color.setStyleSheet("background-color: red;")
        self.legend_layout.addWidget(self.command_color)
        self.legend_layout.addWidget(self.command_label)
        self.legend_layout.addSpacing(10)
        
        self.state_label = QLabel("State Interface")
        self.state_color = QLabel("   ")
        self.state_color.setStyleSheet("background-color: blue;")
        self.legend_layout.addWidget(self.state_color)
        self.legend_layout.addWidget(self.state_label)
        self.legend_layout.addSpacing(10)
        
        self.hardware_label = QLabel("Hardware Node")
        self.hardware_color = QLabel("   ")
        self.hardware_color.setStyleSheet("background-color: green;")
        self.legend_layout.addWidget(self.hardware_color)
        self.legend_layout.addWidget(self.hardware_label)
        
        self.active_label = QLabel("Active Node")
        self.active_color = QLabel("   ")
        self.active_color.setStyleSheet("background-color: lightblue;")
        self.legend_layout.addWidget(self.active_color)
        self.legend_layout.addWidget(self.active_label)
        
        self.inactive_label = QLabel("Inactive Node")
        self.inactive_color = QLabel("   ")
        self.inactive_color.setStyleSheet("background-color: lightgray;")
        self.legend_layout.addWidget(self.active_color)
        self.legend_layout.addWidget(self.active_label)
        
        self.legend_layout.addStretch()
        self.graph_layout.addLayout(self.legend_layout)
        
        self.details_panel = QWidget()
        self.details_layout = QVBoxLayout()
        self.details_panel.setLayout(self.details_layout)
        
        self.message_table = QTableWidget()
        self.message_table.setColumnCount(7)
        self.message_table.setHorizontalHeaderLabels([
            "Start Node", "End Node", "Command Interface", 
            "State Interface", "Hardware Node", "Metadata", "Timestamp"
        ])
        
        self.message_table.horizontalHeader().setSectionResizeMode(QHeaderView.Interactive)
        self.message_table.horizontalHeader().setStretchLastSection(True)
        
        self.details_layout.addWidget(self.message_table)
        
        self.main_layout.addWidget(self.graph_panel, 1)  
        self.main_layout.addWidget(self.details_panel, 1)  
        
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_graph)
        self.update_timer.start(100)
        
        self.node_positions = {}
        
        self.node = None
        self.initialized = False
        
        self.scene.setSceneRect(-200, -200, 400, 400)
        
        self.scene.parent_widget = self
        
        self.add_instruction_text()
    
    def add_instruction_text(self):

        self.instruction_text = QGraphicsTextItem("No connections detected.\nWaiting for NodeConnection messages...")
        self.instruction_text.setFont(QFont("Arial", 12))
        text_rect = self.instruction_text.boundingRect()
        self.instruction_text.setPos(-text_rect.width() / 2, -text_rect.height() / 2)
        self.scene.addItem(self.instruction_text)
    
    def initialize_ros(self):
        if not self.initialized:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = NodeGraphNode(self.update_connection)
            self.initialized = True
            
            self.spin_timer = QTimer()
            self.spin_timer.timeout.connect(self.spin_once)
            self.spin_timer.start(100)

    def spin_once(self):
        if self.node is not None and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0)

    def shutdown_ros(self):
        # Shutdown ROS node and timer
        if self.initialized:
            self.spin_timer.stop()
            self.node.destroy_node()
            self.initialized = False

    def update_connection(self, conn_msg, topic):
        # Update graph 
        with self.mutex:
            start = conn_msg.start
            end = conn_msg.end
            current_time = time.time()
            
            if conn_msg.command_interface:
                connection_type = "command"
            elif conn_msg.state_interface:
                connection_type = "state"
            else:
                connection_type = "default"
            
            if start not in self.nodes:
                self.nodes[start] = GraphNode(start)
            
            if end not in self.nodes:
                self.nodes[end] = GraphNode(end, conn_msg.is_hardware)
            elif conn_msg.is_hardware:
                self.nodes[end].is_hardware = True
            
            self.nodes[start].last_active_time = current_time
            self.nodes[start].is_active = True
            self.nodes[end].is_active = True
            
            key = f"{start}_{end}"
            self.connections[key] = {
                'start': start,
                'end': end,
                'type': connection_type,
                'metadata': conn_msg.metadata,
                'timestamp': current_time,
                'command_interface': conn_msg.command_interface,
                'state_interface': conn_msg.state_interface,
                'is_hardware': conn_msg.is_hardware
            }
            
            if end not in [c[0] for c in self.nodes[start].connections]:
                self.nodes[start].connections.append((end, connection_type))
            
            self.update_message_table()

    def update_message_table(self):
        # Update message table
        self.message_table.setRowCount(len(self.connections))
        
        for row, (_, conn) in enumerate(sorted(self.connections.items(), key=lambda x: x[1]['timestamp'], reverse=True)):
            self.message_table.setItem(row, 0, QTableWidgetItem(str(conn['start'])))
            self.message_table.setItem(row, 1, QTableWidgetItem(str(conn['end'])))
            self.message_table.setItem(row, 2, QTableWidgetItem(str(conn['command_interface'])))
            self.message_table.setItem(row, 3, QTableWidgetItem(str(conn['state_interface'])))
            self.message_table.setItem(row, 4, QTableWidgetItem(str(conn['is_hardware'])))
            self.message_table.setItem(row, 5, QTableWidgetItem(str(conn['metadata'])))
            
            timestamp_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(conn['timestamp']))
            self.message_table.setItem(row, 6, QTableWidgetItem(timestamp_str))
        
        self.message_table.resizeColumnsToContents()

    def update_graph(self):
        # Update graph nodes and connections
        with self.mutex:
            if self.connections and not self.node_views:
                if hasattr(self, 'instruction_text') and self.instruction_text in self.scene.items():
                    self.scene.removeItem(self.instruction_text)
                    delattr(self, 'instruction_text')
                
                self.create_initial_layout()
            
            self.update_nodes_and_connections()

    def create_initial_layout(self):
        # Initial layout for nodes
        self.scene.clear()
        self.node_views = {}
        self.connection_views = []
        
        num_nodes = len(self.nodes)
        radius = 150
        center_x, center_y = 0, 0
        
        for i, (name, node) in enumerate(self.nodes.items()):
            angle = 2 * math.pi * i / num_nodes
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            node_view = GraphNodeView(node, self.scene)
            node_view.setPos(x - 30, y - 30)  
            self.scene.addItem(node_view)
            self.node_views[name] = node_view
        
        self.update_connections()

    def highlight_connection_row(self, connection_data):
        # Highlight row in the message table
        if not connection_data:
            return
        
        for row in range(self.message_table.rowCount()):
            start_node = self.message_table.item(row, 0).text()
            end_node = self.message_table.item(row, 1).text()
            
            if (start_node == connection_data['start'] and 
                end_node == connection_data['end']):
                for col in range(self.message_table.columnCount()):
                    item = self.message_table.item(row, col)
                    item.setBackground(QBrush(QColor(255, 255, 0, 100))) 
                
                self.message_table.scrollToItem(item)
                break
    
    def reset_table_highlight(self):
        # Reset highlighting in the message table
        for row in range(self.message_table.rowCount()):
            for col in range(self.message_table.columnCount()):
                item = self.message_table.item(row, col)
                item.setBackground(QBrush(QColor(255, 255, 255, 0))) 

    def update_nodes_and_connections(self):
        # Update node and connection status and visibility
        current_time = time.time()
        
        for name, node in self.nodes.items():
            if current_time - node.last_active_time > self.node_inactivity_threshold:
                node.is_active = False
            
            if name in self.node_views:
                node_view = self.node_views[name]
                if not node.is_active:
                    node_view.setBrush(QBrush(QColor('darkgray' if node.is_hardware else 'lightgray')))
                else:
                    node_view.setBrush(QBrush(QColor('green' if node.is_hardware else 'lightblue')))
        
        for name, node in self.nodes.items():
            if name not in self.node_views:
                node_view = GraphNodeView(node, self.scene)
                node_view.setPos(0, 0) 
                self.scene.addItem(node_view)
                self.node_views[name] = node_view
                
                self.arrange_nodes_in_circle()
        
        self.update_connections()
    
    def arrange_nodes_in_circle(self):
        num_nodes = len(self.node_views)
        radius = 150
        center_x, center_y = 0, 0
        
        for i, (name, node_view) in enumerate(self.node_views.items()):
            angle = 2 * math.pi * i / num_nodes
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            node_view.setPos(x - 30, y - 30)  
    
    def update_connections(self):
        # Update connection between nodes
        for conn_view in self.connection_views:
            if conn_view in self.scene.items():
                self.scene.removeItem(conn_view)
        self.connection_views = []
        
        for conn_data in self.connections.values():
            source_name = conn_data['start']
            target_name = conn_data['end']
            
            if source_name in self.node_views and target_name in self.node_views:
                source_view = self.node_views[source_name]
                target_view = self.node_views[target_name]
                
                connection_view = GraphConnectionView(
                    source_view, 
                    target_view, 
                    conn_data['type'],
                    conn_data['metadata'],
                    conn_data['timestamp'],
                    conn_data 
                )
                self.scene.addItem(connection_view)
                self.connection_views.append(connection_view)

class NodeGraphPlugin(Plugin):
    # RQT plugin for node graph visualization
    def __init__(self, context):
        super(NodeGraphPlugin, self).__init__(context)
        self.setObjectName('NodeGraphPlugin')

        self._widget = NodeGraphWidget()
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        
        self._widget.initialize_ros()

    def shutdown_plugin(self):
        self._widget.shutdown_ros()

def main():
    rclpy.init(args=None)
    
    from python_qt_binding.QtWidgets import QApplication
    import sys
    
    app = QApplication(sys.argv)
    widget = NodeGraphWidget()
    widget.initialize_ros()
    widget.show()
    
    app.exec_()
    
    widget.shutdown_ros()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
