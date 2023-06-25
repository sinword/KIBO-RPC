package PathCaculation;

import java.util.Map;
import Basic.Cuboid;
import Basic.Transform;

public interface IMapConfig {
    Map<Integer, Transform> getTransformMap();
    Cuboid[] getAllKOZs();
}

