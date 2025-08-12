from mcp.server.fastmcp import FastMCP, Context

import logging
from dataclasses import dataclass
from contextlib import asynccontextmanager
from typing import AsyncIterator, Dict, Any, List
import carla

logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

logger = logging.getLogger("CarlaMCPServer")

@dataclass
class CarlaConnection:
    """Class representing a connection to the Carla server."""
    host: str
    port: int

    def __init__(self, host: str = "localhost", port: int = 2000):
        """Initialize the CarlaConnection with host and port."""
        self.host = host
        self.port = port
        self.client = None
        self.world = None
        self.map = None
        self.blueprints = None
        self.vehicles = []

    def connect(self) -> None:
        """Connect to the Carla server."""
        logger.info(f"Connecting to Carla server at {self.host}:{self.port}")
        
        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(10.0)
            logger.info("Connected to Carla server")
        except Exception as e:
            logger.error(f"Failed to connect to Carla server: {e}")
            raise
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.blueprints = self.world.get_blueprint_library()
        self.vehicles = []
        self.sensors = []
        self.actors = []
        self.vehicles = []

    def disconnect(self) -> None:
        """Disconnect from the Carla server."""
        logger.info("Disconnecting from Carla server")
        try:
            if self.client:
                self.client.apply_batch_sync([carla.command.DestroyActor(x) for x in self.vehicles])
                self.client.apply_batch_sync([carla.command.DestroyActor(x) for x in self.sensors])
                self.client.apply_batch_sync([carla.command.DestroyActor(x) for x in self.actors])
                self.client = None
                logger.info("Disconnected from Carla server")
            else:
                logger.warning("No active connection to disconnect")
        except Exception as e:
            logger.error(f"Failed to disconnect from Carla server: {e}")
            raise
       
    def disconnect_all(self) -> None:
        """Disconnect all actors."""
        logger.info("Disconnecting all actors")
        try:
            if self.client:
                self.client.apply_batch_sync([carla.command.DestroyActor(x) for x in self.vehicles])
                self.client.apply_batch_sync([carla.command.DestroyActor(x) for x in self.sensors])
                self.client.apply_batch_sync([carla.command.DestroyActor(x) for x in self.actors])
                logger.info("Disconnected all actors")
            else:
                logger.warning("No active connection to disconnect")
        except Exception as e:
            logger.error(f"Failed to disconnect all actors: {e}")
            raise
        self.vehicles = []
        self.sensors = []
        self.actors = []
        self.vehicles = []
        self.client = None
        self.world = None
        self.map = None
        self.blueprints = None
    
    def get_map(self) -> str:
        """Get the current map name."""
        if self.map:
            return self.map.name
        else:
            logger.warning("No map loaded")
            return None
        return self.map.name
    def get_blueprints(self) -> List[str]:
        """Get the list of available blueprints."""
        if self.blueprints:
            return [bp.id for bp in self.blueprints.filter("vehicle.*")]
        else:
            logger.warning("No blueprints loaded")
            return []
    def get_vehicles(self) -> List[str]:
        """Get the list of vehicles in the world."""
        if self.vehicles:
            return [vehicle.id for vehicle in self.vehicles]
        else:
            logger.warning("No vehicles loaded")
            return []
    def get_sensors(self) -> List[str]:
        """Get the list of sensors in the world."""
        if self.sensors:
            return [sensor.id for sensor in self.sensors]
        else:
            logger.warning("No sensors loaded")
            return []
    def get_actors(self) -> List[str]:
        """Get the list of actors in the world."""
        if self.actors:
            return [actor.id for actor in self.actors]
        else:
            logger.warning("No actors loaded")
            return []
    def send_command(self, command: str) -> None:
        """Send a command to the Carla server."""
        logger.info(f"Sending command to Carla server: {command}")
        try:
            self.client.apply_batch_sync([carla.command.ExecuteCommand(command)])
            logger.info("Command sent successfully")
        except Exception as e:
            logger.error(f"Failed to send command to Carla server: {e}")
            raise
    def get_world_snapshot(self) -> carla.WorldSnapshot:
        """Get the current world snapshot."""
        if self.world:
            return self.world.get_snapshot()
        else:
            logger.warning("No world loaded")
            return None
    def get_map_name(self) -> str:
        """Get the current map name."""
        if self.map:
            return self.map.name
        else:
            logger.warning("No map loaded")
            return None
   
@asynccontextmanager
async def server_lifespan(server: FastMCP) -> AsyncIterator[Dict[str, Any]]:
    """Lifespan context manager for the FastMCP server."""
    try:
        logger.info("Starting FastMCP server")
        try:
            carla = get_carla_connection()
            logger.info("Sucessfully connected to Carla server")
        except Exception as e:
            logger.error(f"Failed to connect to Carla server: {e}")
        yield {}
    finally:
            global _carla_connection
            if _carla_connection:
                logger.info("Disconnecting from Carla on shutdown")
                _carla_connection.disconnect()
            logger.info("CarlaMCP server shut down")


mcp = FastMCP(
    "CarlaMCP",
    lifespan=server_lifespan
)

_carla_connection = None

def get_carla_connection():
    """Get a connection to the Carla server."""
    global _carla_connection
    if _carla_connection is None:
        try:
            _carla_connection = CarlaConnection()
            _carla_connection.connect()
            logger.info("Created new persistent connection to Carla")
        except Exception as e:
            logger.error(f"Failed to create connection to Carla: {e}")
            raise
    else:
        try:
            _carla_connection.connect()
            logger.info("Reconnected to Carla")
        except Exception as e:
            logger.error(f"Failed to reconnect to Carla: {e}")
            raise
    return _carla_connection


@mcp.tool()
def destroy_all_actors(ctx: Context) -> bool:
    """Destroy all actors in the Carla world."""
    logger.info("Destroying all actors")
    try:
        carla = get_carla_connection()
        carla.disconnect_all()
        logger.info("All actors destroyed")
    except Exception as e:
        logger.error(f"Failed to destroy all actors: {e}")
        raise
    return True

@mcp.tool()
def get_map_name(ctx: Context) -> str:
    """Get the name of the current map."""
    logger.info("Getting map name")
    try:
        carla = get_carla_connection()
        map_name = carla.get_map_name()
        logger.info(f"Map name: {map_name}")
    except Exception as e:
        logger.error(f"Failed to get map name: {e}")
        raise
    return map_name

@mcp.tool()
def get_blueprints() -> List[str]:
    """Get the list of available blueprints."""
    logger.info("Getting blueprints")
    try:
        carla = get_carla_connection()
        blueprints = carla.get_blueprints()
        logger.info(f"Blueprints: {blueprints}")
    except Exception as e:
        logger.error(f"Failed to get blueprints: {e}")
        raise
    return blueprints


def main():
    """Run the MCP server"""
    mcp.run()

if __name__ == "__main__":
    main()