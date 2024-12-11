This video demonstrates the **Semantic Mapper Package**, which identifies and maps objects within a simulated environment using instance segmentation and depth perception. The package creates 

## Processes Running

- **semantic_mapper**  
    - Identifies objects in the environment and updates their locations in the semantic map database.  
- **object_recognition**  
    - Uses vision-based algorithms to detect and classify objects in the environment.  
- **map_database**  
    - Maintains a semantic map that associates object classes with their respective positions.  
- **interpreter**  
    - Provides high-level tasks for the robots based on the updated semantic map.  
- **visualizer**  
    - Renders the semantic map and detected objects for visualization.

## Environment Used

<p align="center">
<img src="semantic_mapping.gif" alt="Semantic Mapping Environment" width="500" height="300">
<br>
**AWS RoboWare Small Warehouse world** with dynamic obstacles and objects for detection.
</p>