#include "libavoid/libavoid.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "helpers.h"

/*
 * Test nudging of many routes at once. It checks following issue https://github.com/Aksem/adaptagrams/issues/24 .
 * Reproducible example from https://github.com/Aksem/libavoid-js/issues/23
 * */


const int CONNECTIONPIN_INCOMING = 111;

class NudgingOrthogonalRouter : public ::testing::Test {
protected:
    void SetUp() override {
        router = new Avoid::Router(Avoid::OrthogonalRouting);
        router->setRoutingParameter(Avoid::RoutingParameter::shapeBufferDistance, 30);
        router->setRoutingParameter(Avoid::RoutingParameter::idealNudgingDistance, 10);
        router->setRoutingOption(Avoid::RoutingOption::nudgeOrthogonalSegmentsConnectedToShapes, true);
        router->setRoutingOption(Avoid::RoutingOption::nudgeOrthogonalTouchingColinearSegments, false);
        router->setRoutingOption(Avoid::RoutingOption::performUnifyingNudgingPreprocessingStep, true);
        router->setRoutingOption(Avoid::RoutingOption::nudgeSharedPathsWithCommonEndPoint, true);
    }

    void TearDown() override {
        delete router;
    }

    // , unsigned int shapeId, unsigned int connectionId, unsigned int connectionId2 = 0
    Avoid::ShapeRef* addChild(Avoid::Point topLeft, Avoid::Point bottomRight) {
        Avoid::Rectangle childRectangle(topLeft, bottomRight);
        Avoid::ShapeRef *childShape = new Avoid::ShapeRef(router, childRectangle);
        return childShape;
    }

    Avoid::ConnRef*  connectShapes(Avoid::ShapeRef *shape1, Avoid::ShapeRef *shape2) {
        Avoid::ShapeConnectionPin *shapeConnectionPinSrc1 = new Avoid::ShapeConnectionPin(shape1, 1,
                               0.5, 0.5, true, 0.0, Avoid::ConnDirAll);
        shapeConnectionPinSrc1->setExclusive(false);
        Avoid::ConnEnd connRefSrcConnEnd(shape1, 1);

        Avoid::ShapeConnectionPin *shapeConnectionPinDest1 = new Avoid::ShapeConnectionPin(shape2, 1,
                               0.5, 0.5, true, 0.0, Avoid::ConnDirAll);
        shapeConnectionPinDest1->setExclusive(false);
        Avoid::ConnEnd connRefDestConnEnd(shape2, 1);

        Avoid::ConnRef *connection = new Avoid::ConnRef(router, connRefSrcConnEnd, connRefDestConnEnd);
        return connection;
    }

    Avoid::Router *router;
};

TEST_F(NudgingOrthogonalRouter, Multiple) {
    int w = 120;
    int h = 120;

    Avoid::ShapeRef *shapeRef1 = addChild({ 0, 680 }, { 0.0 + w, 700.0 + h });
    Avoid::ShapeRef *shapeRef2 = addChild({ 320, 1055 }, { 320.0 + w, 1055.0 + h });
    Avoid::ShapeRef *shapeRef3 = addChild({ 320, 1255 }, { 320.0 + w, 1255.0 + h });
    Avoid::ShapeRef *shapeRef4 = addChild({ 320, 1255 }, { 320.0 + w, 1255.0 + h });
    Avoid::ShapeRef *shapeRef5 = addChild({ 320, 1565 }, { 320.0 + w, 1565.0 + h });
    Avoid::ShapeRef *shapeRef6 = addChild({ 320, 1735 }, { 320.0 + w, 1735.0 + h });
    Avoid::ShapeRef *shapeRef7 = addChild({ 320, 1905 }, { 320.0 + w, 1905.0 + h });
    Avoid::ShapeRef *shapeRef8 = addChild({ 320, 2075 }, { 320.0 + w, 2075.0 + h });
    Avoid::ShapeRef *shapeRef9 = addChild({ 640, 1605 }, { 640.0 + w, 1605.0 + h });
    Avoid::ShapeRef *shapeRef10 = addChild({ 320, 2245 }, { 320.0 + w, 2245.0 + h });
    Avoid::ShapeRef *shapeRef11 = addChild({ 640, 25 }, { 640.0 + w, 25.0 + h });
    Avoid::ShapeRef *shapeRef12 = addChild({ 320, 25 }, { 320.0 + w, 25.0 + h });
    Avoid::ShapeRef *shapeRef13 = addChild({ 0, 1900 }, { 0.0 + w, 1900.0 + h });
    Avoid::ShapeRef *shapeRef14 = addChild({ 960, 2560 }, { 960.0 + w, 2560.0 + h });
    Avoid::ShapeRef *shapeRef15 = addChild({ 640, 2522.5 }, { 640.0 + w, 2522.5 + h });
    Avoid::ShapeRef *shapeRef16 = addChild({ 640, 362.5 }, { 640.0 + w, 362.5 + h });
    Avoid::ShapeRef *shapeRef17 = addChild({ 960, 532.5 }, { 960.0 + w, 532.5 + h });
    Avoid::ShapeRef *shapeRef18 = addChild({ 640, 532.5 }, { 640.0 + w, 532.5 + h });
    Avoid::ShapeRef *shapeRef19 = addChild({ 0, 510 }, { 0.0 + w, 510.0 + h });
    Avoid::ShapeRef *shapeRef20 = addChild({ 0, 680 }, { 0.0 + w, 680.0 + h });
    Avoid::ShapeRef *shapeRef21 = addChild({ 640, 702.5 }, { 640.0 + w, 702.5 + h });
    Avoid::ShapeRef *shapeRef22 = addChild({ 1280, 1737 }, { 1280.0 + w, 1737.0 + h });
    Avoid::ShapeRef *shapeRef23 = addChild({ 1280, 2950 }, { 1280.0 + w, 2950.0 + h });
    Avoid::ShapeRef *shapeRef24 = addChild({ 0, 1090 }, { 0.0 + w, 1090.0 + h });
    Avoid::ShapeRef *shapeRef25 = addChild({ 640, 872 }, { 640.0 + w, 872.0 + h });
    Avoid::ShapeRef *shapeRef26 = addChild({ 320, 1245 }, { 320.0 + w, 1245.0 + h });
    Avoid::ShapeRef *shapeRef27 = addChild({ 0, 1260 }, { 0.0 + w, 1260.0 + h });

    std::vector<Avoid::ShapeRef *> allShapes = { shapeRef1, shapeRef2, shapeRef3, shapeRef4, shapeRef5, shapeRef6, shapeRef7, shapeRef8, shapeRef9, shapeRef10, shapeRef11, shapeRef12, shapeRef13, shapeRef14, shapeRef15, shapeRef16, shapeRef17, shapeRef18, shapeRef19, shapeRef20, shapeRef21, shapeRef22, shapeRef23, shapeRef24, shapeRef25, shapeRef26, shapeRef27 };
    std::vector<Avoid::ShapeRef *> shapes = { shapeRef1, shapeRef2, shapeRef3, shapeRef4, shapeRef5, shapeRef6, shapeRef7, shapeRef8, shapeRef9, shapeRef10, shapeRef11, shapeRef12 };

    int numConnections = 2;
    std::vector<Avoid::ConnRef *> connections;
    for (int i = 0; i < shapes.size(); i++)
    {
        // create two way connections between shapes
        for (int j = 0; j < shapes.size(); j++)
        {
            // Create multiple connections between shape[i] and shape[j]
            for (int k = 0; k < numConnections; k++)
            {
                connections.push_back(connectShapes(shapes[i], shapes[j]));
            }
        }
    }

    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/NudgingOrthogonalRouter_Multiple");

    for (Avoid::ConnRef *conn : connections)
    {
        router->deleteConnector(conn);
    }

    for (Avoid::ShapeRef *shapeRef : allShapes)
    {
        router->deleteShape(shapeRef);
    }
}
