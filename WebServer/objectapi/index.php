<?php
/*
 * Typically this data would be in a global database.   Hardcoded for proof of concept's simplicity.
 *
 * All measurements are in meters.
 *
 * stl_file paths should be equivalent to the object's ID initially.  If upgraded STL files are then released for an existing object add -1, -2 and so forth to the end ([objectid]-1 being the second version of the STL file).
 */


$objects = array();

//3x sample products are below

//Sadly this product is discontinued but you can purchase the other two for testing.  Amazon links below.

$objects["80043752830000000A104805"] = new stdClass();
$objects["80043752830000000A104805"]->size = new stdClass();
$objects["80043752830000000A104805"]->transformations = new stdClass();
$objects["80043752830000000A104805"]->grasp = new stdClass();
$objects["80043752830000000A104799"]->color = new stdClass();
//All measurements are in meters

$objects["80043752830000000A104805"]->nickname = "Cup"; //used for QC / error / log msgs
$objects["80043752830000000A104805"]->stl_file = "80043752830000000A104805"; //STL filename (without .stl)

#transformation relative to leading point in meters; rotation matrix used to transform relative to orientation
$objects["80043752830000000A104805"]->transformation->x_offset = -0.055;
$objects["80043752830000000A104805"]->transformation->y_offset = 0;


#size of object in meters
$objects["80043752830000000A104805"]->size->l = 0.08; #y axis
$objects["80043752830000000A104805"]->size->w = 0.14; #x axis
$objects["80043752830000000A104805"]->size->h = 0.095; #z axis

#grasping pose (relative to leading point) and object's orientation (not robots)
$objects["80043752830000000A104805"]->grasp->x_offset = 0.05;
$objects["80043752830000000A104805"]->grasp->y_offset = 0;
$objects["80043752830000000A104805"]->grasp->z_offset = 0.12; #a positive value is above the object in meters; this doesn't include the grippers length!
#grasping position
$objects["80043752830000000A104805"]->grasp->yaw = 0;

#color of object in RGB format (values must be numeric) for displaying in MoveIt

$objects["80043752830000000A104805"]->color->r = 0.191;
$objects["80043752830000000A104805"]->color->g = 0.191;
$objects["80043752830000000A104805"]->color->b = 0.191;

//-------
//Can be purchased at: http://www.amazon.com/gp/product/B008OLKVEW?psc=1&redirect=true&ref_=oh_aui_detailpage_o05_s00

$objects["80043752830000000A104799"] = new stdClass();
$objects["80043752830000000A104799"]->size = new stdClass();
$objects["80043752830000000A104799"]->transformations = new stdClass();
$objects["80043752830000000A104799"]->grasp = new stdClass();
$objects["80043752830000000A104799"]->color = new stdClass();

$objects["80043752830000000A104799"]->nickname = "Kettle"; //used for QC / error / log msgs
$objects["80043752830000000A104799"]->stl_file = "80043752830000000A104799"; //STL filename (without .stl)

#transformation relative to leading point in meters; rotation matrix used to transform relative to orientation
$objects["80043752830000000A104799"]->transformation->x_offset = -0.10;
$objects["80043752830000000A104799"]->transformation->y_offset = 0;

#size of object in meters
$objects["80043752830000000A104799"]->size->l = 0.192; #y axis
$objects["80043752830000000A104799"]->size->w = 0.210; #x axis
$objects["80043752830000000A104799"]->size->h = 0.229; #z axis

#grasping pose (relative to leading point) and object's orientation (not robots)
$objects["80043752830000000A104799"]->grasp->x_offset = -0.10;
$objects["80043752830000000A104799"]->grasp->y_offset = 0;
$objects["80043752830000000A104799"]->grasp->z_offset = 0.25; #a positive value is above the object in meters; this doesn't include the grippers length!

#color of object in RGB format (values must be numeric) for displaying in MoveIt

$objects["80043752830000000A104799"]->color->r = 0.69;
$objects["80043752830000000A104799"]->color->g = 0.68;
$objects["80043752830000000A104799"]->color->b = 0.67;

#grasping position
$objects["80043752830000000A104799"]->grasp->yaw = -1 * pi() / 2;

//-------
//Can be purchased (1-1/2 Quart option) at: http://www.amazon.com/gp/product/B00008CM69?psc=1&redirect=true&ref_=oh_aui_detailpage_o05_s01

$objects["201504208040100001070770"] = new stdClass();
$objects["201504208040100001070770"]->size = new stdClass();
$objects["201504208040100001070770"]->transformations = new stdClass();
$objects["201504208040100001070770"]->grasp = new stdClass();
//All measurements are in meters

$objects["201504208040100001070770"]->nickname = "Pot"; //used for QC / error / log msgs
$objects["201504208040100001070770"]->stl_file = "201504208040100001070770"; //STL filename (without .stl)

#transformation relative to leading point in meters; rotation matrix used to transform relative to orientation
$objects["201504208040100001070770"]->transformation->x_offset = -0.15;
$objects["201504208040100001070770"]->transformation->y_offset = 0;


#size of object in meters
$objects["201504208040100001070770"]->size->l = 0.189; #y axis
$objects["201504208040100001070770"]->size->w = 0.346; #x axis
$objects["201504208040100001070770"]->size->h = 0.107; #z axis

#color of object in RGB format (values must be numeric) for displaying in MoveIt

$objects["201504208040100001070770"]->color->r = 0.33;
$objects["201504208040100001070770"]->color->g = 0.33;
$objects["201504208040100001070770"]->color->b = 0.33;


#grasping pose (relative to leading point) and object's orientation (not robots)
$objects["201504208040100001070770"]->grasp->x_offset = 0.05;
$objects["201504208040100001070770"]->grasp->y_offset = 0;
$objects["201504208040100001070770"]->grasp->z_offset = 0.12; #a positive value is above the object in meters; this doesn't include the grippers length!
#grasping position
$objects["201504208040100001070770"]->grasp->yaw = 0;

if(isset($_GET['objectapi_id'])) {
    header('Content-Type: application/json');
    if(!array_key_exists($_GET['objectapi_id'],$objects)) {
        echo json_encode("does not exist");
        exit;
    }
    echo json_encode($objects[$_GET['objectapi_id']]);
    exit;
}