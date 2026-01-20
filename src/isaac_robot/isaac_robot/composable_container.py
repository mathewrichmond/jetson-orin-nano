#!/usr/bin/env python3
"""
Composable Node Container for Python Nodes
Runs multiple ROS 2 nodes in a single process to enable zero-copy intra-process communication.

This achieves the same benefits as C++ composable nodes:
- Zero-copy message passing between nodes in the same process
- Reduced latency and CPU usage
- Lower memory overhead

Usage:
    ros2 run isaac_robot composable_container --ros-args -p composable_nodes:='[...]'
"""

# Standard library
import importlib
import json
import os
from pathlib import Path
import sys
from typing import Dict, List, Optional

# Third-party
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor, TimeoutException
from rclpy.node import Node
from rclpy.parameter import Parameter


# Add ROS 2 workspace paths to sys.path for dynamic imports
def _add_ros2_workspace_paths():
    """Add ROS 2 workspace install paths to sys.path for module imports"""
    # Common ROS 2 workspace locations
    workspace_paths = [
        Path.home() / "ros2_ws" / "install",
        Path("/opt/ros/humble"),
    ]

    for workspace in workspace_paths:
        if workspace.exists():
            # Add each package's Python path
            for package_dir in workspace.iterdir():
                if package_dir.is_dir():
                    # Standard Python package location
                    python_path = package_dir / "local" / "lib" / "python3.10" / "dist-packages"
                    if python_path.exists() and str(python_path) not in sys.path:
                        sys.path.insert(0, str(python_path))

                    # Alternative location
                    lib_path = package_dir / "lib" / package_dir.name
                    if lib_path.exists() and str(lib_path.parent) not in sys.path:
                        sys.path.insert(0, str(lib_path.parent))


# Initialize ROS 2 workspace paths
_add_ros2_workspace_paths()


class ComposableNodeContainer(Node):
    """Container that runs multiple nodes in a single process with intra-process communication"""

    # Use class-level dict to store executor to avoid attribute access issues
    _executor_storage = {}

    def __init__(self, container_name: str = "composable_container"):
        super().__init__(container_name)
        # Use object.__setattr__ to bypass any potential Node class overrides
        object.__setattr__(self, "nodes", [])
        object.__setattr__(self, "running", False)
        # Store executor in class dict using instance ID as key
        instance_id = id(self)
        object.__setattr__(self, "_instance_id", instance_id)
        ComposableNodeContainer._executor_storage[instance_id] = None

        # Declare parameter for composable node configurations
        # Use JSON string since ROS 2 parameters don't support nested lists/dicts
        self.declare_parameter("composable_nodes_json", "[]")

        self.get_logger().info(
            f"ComposableNodeContainer '{container_name}' initialized (instance_id: {self._instance_id})"
        )

    @property
    def executor(self):
        """Get executor from storage"""
        try:
            instance_id = object.__getattribute__(self, "_instance_id")
        except AttributeError:
            # Fallback if _instance_id not set yet (shouldn't happen, but be safe)
            instance_id = id(self)
        return ComposableNodeContainer._executor_storage.get(instance_id)

    @executor.setter
    def executor(self, value):
        """Set executor in storage"""
        try:
            instance_id = object.__getattribute__(self, "_instance_id")
        except AttributeError:
            # Fallback if _instance_id not set yet
            instance_id = id(self)
            object.__setattr__(self, "_instance_id", instance_id)
        ComposableNodeContainer._executor_storage[instance_id] = value

    def load_components_from_config(self):
        """Load components from parameter configuration"""
        try:
            # Get JSON string parameter and parse it
            composable_nodes_json = self.get_parameter("composable_nodes_json").value
            self.get_logger().info(
                f"Got composable_nodes_json parameter (length: {len(composable_nodes_json) if composable_nodes_json else 0})"
            )

            if not composable_nodes_json or composable_nodes_json == "[]":
                self.get_logger().warn("No composable nodes configured (empty JSON)")
                return

            composable_nodes_config = json.loads(composable_nodes_json)
            self.get_logger().info(f"Parsed JSON config: {len(composable_nodes_config)} nodes")
        except Exception as e:
            self.get_logger().error(
                f"No composable_nodes_json parameter found or invalid JSON: {e}"
            )
            # Standard library
            import traceback

            self.get_logger().error(traceback.format_exc())
            return

        if not composable_nodes_config:
            self.get_logger().warn("No composable nodes configured (empty config)")
            return

        if not isinstance(composable_nodes_config, list):
            self.get_logger().error(
                f"composable_nodes_json must be a JSON array, got {type(composable_nodes_config)}"
            )
            return

        self.get_logger().info(f"Loading {len(composable_nodes_config)} composable nodes")

        for node_config in composable_nodes_config:
            try:
                package = node_config.get("package")
                module = node_config.get("module")
                class_name = node_config.get("class")
                node_name = node_config.get("name")
                namespace = node_config.get("namespace", "")
                parameters = node_config.get("parameters", {})

                if not all([package, module, class_name]):
                    self.get_logger().error(f"Invalid node config: missing package/module/class")
                    continue

                # Load the component
                node = self.load_component(
                    package=package,
                    module=module,
                    class_name=class_name,
                    node_name=node_name,
                    namespace=namespace,
                    parameters=parameters,
                )

            except Exception as e:
                self.get_logger().error(f"Failed to load node from config: {e}")
                continue

    def load_component(
        self,
        package: str,
        module: str,
        class_name: str,
        node_name: Optional[str] = None,
        namespace: Optional[str] = None,
        parameters: Optional[Dict] = None,
    ) -> Node:
        """
        Load a node component dynamically

        Args:
            package: Python package name (e.g., 'realsense_camera')
            module: Module name within package (e.g., 'realsense_camera_node')
            class_name: Class name of the node (e.g., 'RealSenseCameraNode')
            node_name: Optional node name override
            namespace: Optional namespace
            parameters: Optional parameters dict

        Returns:
            The loaded node instance
        """
        try:
            # Import the module
            module_path = f"{package}.{module}"
            self.get_logger().info(f"Loading component: {module_path}.{class_name}")
            self.get_logger().info(f"  Package: {package}, Module: {module}, Class: {class_name}")
            self.get_logger().info(f"  Node name: {node_name}, Namespace: {namespace}")

            # Import the module
            try:
                node_module = importlib.import_module(module_path)
                self.get_logger().info(f"  ✓ Successfully imported {module_path}")
            except ImportError as e:
                self.get_logger().error(f"  ✗ Failed to import {module_path}: {e}")
                self.get_logger().error(f"  Python path: {sys.path[:5]}")
                raise

            # Get the node class
            try:
                node_class = getattr(node_module, class_name)
                self.get_logger().info(f"  ✓ Found class {class_name}")
            except AttributeError as e:
                self.get_logger().error(f"  ✗ Class {class_name} not found in {module_path}: {e}")
                raise

            # Create node instance with deferred initialization if parameters need to be set
            # Some nodes (like camera node) need parameters set before initialization
            defer_init = parameters is not None and len(parameters) > 0

            # Build kwargs for node instantiation
            init_kwargs = {}
            if defer_init:
                init_kwargs["defer_init"] = True
            if node_name:
                init_kwargs["node_name"] = node_name
            if namespace:
                init_kwargs["namespace"] = namespace

            try:
                # Try to instantiate with all provided parameters
                try:
                    node_instance = node_class(**init_kwargs)
                    if defer_init:
                        self.get_logger().info(
                            f"  ✓ Created {class_name} with deferred initialization"
                        )
                    else:
                        self.get_logger().info(f"  ✓ Created {class_name}")
                except TypeError as e:
                    # Node doesn't support some parameters, try with fewer
                    self.get_logger().debug(
                        f"  Node doesn't accept all parameters, trying simpler instantiation: {e}"
                    )
                    # Try without namespace first (some nodes handle it differently)
                    if "namespace" in init_kwargs:
                        init_kwargs_no_ns = {
                            k: v for k, v in init_kwargs.items() if k != "namespace"
                        }
                        try:
                            node_instance = node_class(**init_kwargs_no_ns)
                            self.get_logger().info(
                                f"  ✓ Created {class_name} (without namespace parameter)"
                            )
                        except TypeError:
                            # Fall back to minimal instantiation
                            node_instance = node_class()
                            self.get_logger().info(
                                f"  ✓ Created {class_name} (minimal instantiation)"
                            )
                    else:
                        # Try minimal instantiation
                        node_instance = node_class()
                        self.get_logger().info(f"  ✓ Created {class_name} (minimal instantiation)")
            except Exception as e:
                self.get_logger().error(f"Failed to instantiate {class_name}: {e}")
                # Standard library
                import traceback

                self.get_logger().error(traceback.format_exc())
                raise

            # Set parameters if provided
            # Note: Parameters should be set before node starts processing
            # For nodes with deferred init, this happens before camera initialization
            if parameters:
                self.get_logger().info(f"Setting {len(parameters)} parameters for {class_name}")
                param_list = []
                for key, value in parameters.items():
                    try:
                        # Check if parameter already declared
                        if node_instance.has_parameter(key):
                            # Update existing parameter
                            param_list.append(Parameter(key, value=value))
                            self.get_logger().debug(f"  - Setting parameter {key} = {value}")
                        else:
                            # Declare new parameter
                            node_instance.declare_parameter(key, value)
                            self.get_logger().debug(f"  - Declaring parameter {key} = {value}")
                    except Exception as e:
                        self.get_logger().warn(f"Could not set parameter {key}: {e}")

                # Set all parameters at once
                if param_list:
                    try:
                        result = node_instance.set_parameters(param_list)
                        # Check if any parameters failed to set
                        for r in result:
                            if not r.successful:
                                self.get_logger().warn(f"Failed to set parameter: {r.reason}")
                        self.get_logger().info(
                            f"Successfully set {len([r for r in result if r.successful])}/{len(param_list)} parameters"
                        )

                        # If node supports deferred initialization, initialize now
                        if defer_init and hasattr(node_instance, "_initialize_cameras"):
                            self.get_logger().info(
                                f"Initializing {class_name} after parameter setup"
                            )
                            try:
                                node_instance._initialize_cameras()
                                self.get_logger().info(f"✓ {class_name} initialization complete")
                            except Exception as e:
                                self.get_logger().error(f"Failed to initialize {class_name}: {e}")
                                # Standard library
                                import traceback

                                self.get_logger().error(traceback.format_exc())
                                raise
                    except Exception as e:
                        self.get_logger().error(f"Could not set parameters: {e}")
                        # Standard library
                        import traceback

                        self.get_logger().error(traceback.format_exc())

            # Add to container
            self.nodes.append(node_instance)
            self.get_logger().info(
                f"✓ Loaded component: {class_name} (node: {node_instance.get_name()})"
            )

            return node_instance

        except Exception as e:
            self.get_logger().error(
                f"Failed to load component {package}.{module}.{class_name}: {e}"
            )
            # Standard library
            import traceback

            self.get_logger().error(traceback.format_exc())
            raise

    def start(self):
        """Start the container and all loaded nodes"""
        if self.running:
            self.get_logger().warn("Container is already running")
            return

        if not self.nodes:
            self.get_logger().warn("No nodes loaded, nothing to start")
            return

        self.get_logger().info(f"Starting container with {len(self.nodes)} nodes")
        self.get_logger().info(
            f"Executor before creation: {self.executor}, type: {type(self.executor)}"
        )

        # Create executor with multiple threads for parallel execution
        # Use one thread per node + one for the container
        num_threads = len(self.nodes) + 1
        try:
            # Get the context from this node
            context = self.context
            self.get_logger().info(
                f"Creating executor with {num_threads} threads, context type: {type(context)}, context: {context}"
            )

            # Create executor - try MultiThreadedExecutor to isolate blocking operations
            # SingleThreadedExecutor was blocking for 300-800ms per spin_once() call
            # MultiThreadedExecutor should help isolate blocking callbacks to separate threads
            self.get_logger().info(f"About to create MultiThreadedExecutor with context: {context}")
            executor_result = None
            try:
                # Use MultiThreadedExecutor with 4 threads to handle blocking callbacks
                # This allows timers to fire on time even if other callbacks are blocking
                executor_result = MultiThreadedExecutor(context=context, num_threads=4)
                self.get_logger().info(
                    f"MultiThreadedExecutor() returned: {executor_result}, type: {type(executor_result)}"
                )
            except Exception as create_error:
                self.get_logger().error(
                    f"Exception during MultiThreadedExecutor creation: {create_error}"
                )
                # Standard library
                import traceback

                self.get_logger().error(traceback.format_exc())
                raise

            # Assign executor using property setter
            self.executor = executor_result
            self.get_logger().info(
                f"After assignment, self.executor: {self.executor}, type: {type(self.executor)}, id: {id(self.executor)}"
            )

            # Verify executor was created
            if self.executor is None:
                self.get_logger().error("Executor is None after creation - this should not happen!")
                self.get_logger().error(f"executor_result was: {executor_result}")
                raise RuntimeError("Executor returned None")

            self.get_logger().info(
                f"Executor created successfully: {type(self.executor)}, id: {id(self.executor)}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to create executor: {e}")
            # Standard library
            import traceback

            self.get_logger().error(traceback.format_exc())
            # Don't set executor to None here - let it remain uninitialized
            return

        # Add container node to executor
        try:
            self.get_logger().info(
                f"Adding container node to executor: {self.executor}, type: {type(self.executor)}"
            )
            if self.executor is None:
                self.get_logger().error(
                    "CRITICAL: Executor is None when trying to add container node!"
                )
                raise RuntimeError("Executor is None")
            self.executor.add_node(self)
            self.get_logger().info(
                f"Container node added successfully, executor still: {self.executor}, id: {id(self.executor)}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to add container node to executor: {e}")
            # Standard library
            import traceback

            self.get_logger().error(traceback.format_exc())
            self.executor = None  # Clear executor on failure
            return

        # Add all component nodes to executor
        for node in self.nodes:
            try:
                self.get_logger().info(
                    f"Adding component node '{node.get_name()}' to executor: {self.executor}, id: {id(self.executor) if self.executor else None}"
                )
                if self.executor is None:
                    self.get_logger().error(
                        f"CRITICAL: Executor became None when adding node '{node.get_name()}'!"
                    )
                    raise RuntimeError("Executor is None")
                self.executor.add_node(node)
                self.get_logger().info(
                    f"  - Added node '{node.get_name()}' to executor, executor still: {self.executor}, id: {id(self.executor)}"
                )
            except Exception as e:
                self.get_logger().error(f"Failed to add node '{node.get_name()}' to executor: {e}")
                # Standard library
                import traceback

                self.get_logger().error(traceback.format_exc())
                # Don't clear executor, continue with other nodes

        self.running = True

        # Final verification before returning
        executor_at_end = self.executor
        executor_id_at_end = id(executor_at_end) if executor_at_end else None
        self.get_logger().info(
            f"✓ Container started successfully (intra-process communication enabled)"
        )
        self.get_logger().info(
            f"Executor at end of start(): {executor_at_end}, type: {type(executor_at_end)}, id: {executor_id_at_end}"
        )

        # Double-check executor is still set
        if executor_at_end is None:
            self.get_logger().error(
                "CRITICAL: Executor became None during start() - this should not happen!"
            )
            raise RuntimeError("Executor was None at end of start()")

        # Store executor ID for debugging
        self._executor_id = executor_id_at_end
        executor_check_again = self.executor
        self.get_logger().info(
            f"Stored executor ID: {self._executor_id}, current executor ID: {id(executor_check_again) if executor_check_again else None}"
        )

    def spin(self):
        """Spin the executor (blocks until shutdown)"""
        self.get_logger().info(
            f"spin() called - executor: {self.executor}, id: {id(self.executor) if self.executor else None}, running: {self.running}"
        )

        # Start container if not already started
        if not self.running:
            self.get_logger().info("Container not running, calling start()...")
            executor_before_start = self.executor
            executor_id_before_start = id(executor_before_start) if executor_before_start else None
            try:
                self.start()
            except Exception as e:
                self.get_logger().error(f"start() raised exception: {e}")
                # Standard library
                import traceback

                self.get_logger().error(traceback.format_exc())
                raise

            executor_after_start = self.executor
            executor_id_after_start = id(executor_after_start) if executor_after_start else None
            self.get_logger().info(
                f"start() returned - executor: {executor_after_start}, id: {executor_id_after_start}, running: {self.running}"
            )
            self.get_logger().info(
                f"Executor changed? before: {executor_before_start} (id: {executor_id_before_start}), after: {executor_after_start} (id: {executor_id_after_start})"
            )
            if hasattr(self, "_executor_id"):
                self.get_logger().info(
                    f"Stored executor ID from start(): {self._executor_id}, current executor ID: {executor_id_after_start}"
                )

        executor_to_check = self.executor
        self.get_logger().info(
            f"After start check - executor: {executor_to_check}, type: {type(executor_to_check)}, id: {id(executor_to_check) if executor_to_check else None}, running: {self.running}"
        )

        # Verify executor is initialized
        if executor_to_check is None:
            self.get_logger().error(
                f"Cannot spin: executor not initialized (executor={executor_to_check}, self.running={self.running})"
            )
            self.get_logger().error(
                f"Container state - nodes: {len(self.nodes)}, executor id: {id(executor_to_check) if executor_to_check else None}"
            )
            # Try to start again
            self.get_logger().warn("Attempting to start container again...")
            self.running = False  # Reset running flag
            self.start()
            executor_to_check = self.executor
            self.get_logger().info(
                f"After retry start - executor: {executor_to_check}, type: {type(executor_to_check)}"
            )
            if executor_to_check is None:
                self.get_logger().error("Executor still not initialized after retry - aborting")
                return

        self.get_logger().info(
            f"Spinning executor: {type(executor_to_check)}, id: {id(executor_to_check)}, nodes: {len(self.nodes)}"
        )

        # Verify nodes are actually in the executor
        try:
            executor_nodes = executor_to_check.get_nodes()
            self.get_logger().info(f"Executor has {len(executor_nodes)} node(s) registered")
            for n in executor_nodes:
                self.get_logger().debug(f"  - Node: {n.get_name()}")
        except Exception as e:
            self.get_logger().warn(f"Could not get executor nodes: {e}")

        try:
            # Use executor.spin() with MultiThreadedExecutor
            # MultiThreadedExecutor handles callbacks in separate threads, preventing blocking
            # This should allow timers to fire on time even if other callbacks are blocking
            self.get_logger().info(
                "Starting MultiThreadedExecutor spin (blocking until shutdown)..."
            )
            self.get_logger().info(
                f"Executor nodes: {len(executor_to_check.get_nodes()) if hasattr(executor_to_check, 'get_nodes') else 'unknown'}"
            )

            # Use executor.spin() which handles threading internally
            # MultiThreadedExecutor will use its thread pool to process callbacks
            # Timers should fire on time regardless of blocking callbacks
            executor_to_check.spin()
        except KeyboardInterrupt:
            self.get_logger().info("Received shutdown signal")
        except Exception as e:
            self.get_logger().error(f"Executor error: {e}")
            # Standard library
            import traceback

            self.get_logger().error(traceback.format_exc())
        finally:
            self.stop()

    def stop(self):
        """Stop the container and all nodes"""
        if not self.running:
            return

        self.get_logger().info("Stopping container...")
        self.running = False

        # Shutdown executor
        executor_to_shutdown = self.executor
        if executor_to_shutdown:
            executor_to_shutdown.shutdown()
            # Clear executor from storage
            ComposableNodeContainer._executor_storage.pop(self._instance_id, None)

        # Destroy all nodes
        for node in self.nodes:
            try:
                node.destroy_node()
            except Exception as e:
                self.get_logger().error(f"Error destroying node {node.get_name()}: {e}")

        # Destroy container node
        self.destroy_node()

        self.get_logger().info("✓ Container stopped")


def main(args=None):
    """Main entry point for composable container"""
    rclpy.init(args=args)

    # Create container
    container = ComposableNodeContainer()

    try:
        # Load components from configuration
        container.load_components_from_config()

        # Start and spin
        container.spin()

    except Exception as e:
        container.get_logger().error(f"Container error: {e}")
        # Standard library
        import traceback

        container.get_logger().error(traceback.format_exc())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
