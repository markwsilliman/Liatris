<?php

require_once $_SERVER['DOCUMENT_ROOT'] . "/db.php";

if(isset($_GET['touch_report_print'])) {
    $tr = new touch_report();
    $tr->print_report();
}

class touch_report {
    private $output;
    private $most_recent_object = false;
    private $orientation_in_radians = false;
    private $orientation_quadrent = false;

    //__construct
    public function __construct() {

        $db = new db();
        $this->most_recent_object = false;
        $result = $db->query("select * from touch order by ID desc limit 100");
        $output = "";
        $output .= "<table border='1'>";
        $output .= "<tr><th>ID</th><th>X</th><th>Y</th><th>TYPE</th><th>DATE</th></tr>";
        $downs = array();
        $downs_index = 0;
        while($row = $db->fetch_array($result)) {
            $output .= "<tr ";
            if($row['type_percent'] == 1) { //type_percent 1 is "down" (as opposed to up or move which could be detected as well in the future)
                    //collect the last 3
                    if($downs_index <= 2 && !$this->down_already_exists($downs,$row['x_percent'],$row['y_percent'])) {
                        $downs[$downs_index] = new stdClass();
                        $downs[$downs_index]->x = $row['x_percent'];
                        $downs[$downs_index]->y = $row['y_percent'];
                        $downs[$downs_index]->timestamp = strtotime($row['date_created']);
                        $downs_index++;
                    }
                //}
            }
            $output .= "><td>" . $row['ID'] . "</td><td>" . $row['x_percent'] . "</td><td>" . $row['y_percent'] . "</td><td>" . $this->type_to_english($row['type_percent']) . "</td><td>" . $row['date_created'] . "</td></tr>";
        }
        $output .= "</table>";

        $this->output = $output;

        if(count($downs) == 3) {
            //check if timestamps are close; logic is that if an object is placed on the captouch all 3 points should be recorded in less than 1 second.   If > 1 second they are likely not related.
            if(abs($downs[0]->timestamp - $downs[1]->timestamp) <= 1 && abs($downs[2]->timestamp - $downs[1]->timestamp) <= 1) {
                $this->most_recent_object = $downs;
                $this->determine_objects_orientation();
            }
        }
    }
    //end __construct

    //down_already_exists
    private function down_already_exists($downs,$x,$y) {
        foreach($downs as $index => $down) {
            if($down->x == $x && $down->y == $y) {
                return true;
            }
        }
        return false;
    }
    //end down_already_exists

    //print_report
    public function print_report() {
        echo $this->output;
    }
    //end print_report

    //most_recent_object
    public function most_recent_object() {
        return $this->most_recent_object;
    }
    //end most_recent_object

    //determine_leading_point_pos
    public function determine_leading_point_pos() {
        return $this->determine_objects_orientation(true);
    }
    //end determine_leading_point_pos

    //object_information
    public function object_information() {
        $leading_pont = $this->most_recent_object[$this->determine_leading_point_pos()];
        $orientation_in_radians = $this->orientation_in_radians;
        $leading_pont->orientation_in_radians = $orientation_in_radians;
        return $leading_pont;
    }
    //end object_information

    //determine_objects_orientation
    public function determine_objects_orientation($return_leading_point = false) {
        if(!$this->most_recent_object) {
            return false;
        }

        //step 1: determine which object is the 'leading'
        //by adding the distance to both other points to the point
        //the highest sum is the farthest
        $this->most_recent_object[0]->distancetobothpoints = $this->distance_between_two_points($this->most_recent_object[0],$this->most_recent_object[1]) + $this->distance_between_two_points($this->most_recent_object[0],$this->most_recent_object[2]);
        $this->most_recent_object[1]->distancetobothpoints = $this->distance_between_two_points($this->most_recent_object[1],$this->most_recent_object[0]) + $this->distance_between_two_points($this->most_recent_object[1],$this->most_recent_object[2]);
        $this->most_recent_object[2]->distancetobothpoints = $this->distance_between_two_points($this->most_recent_object[2],$this->most_recent_object[0]) + $this->distance_between_two_points($this->most_recent_object[2],$this->most_recent_object[1]);

        $leading_point = 0;
        if($this->most_recent_object[1]->distancetobothpoints > $this->most_recent_object[$leading_point]->distancetobothpoints) {
            $leading_point = 1;
        }
        if($this->most_recent_object[2]->distancetobothpoints > $this->most_recent_object[$leading_point]->distancetobothpoints) {
            $leading_point = 2;
        }

        //step 2: find middle of two closer points and find orientation of path from middle to farthest

        //which points arent the leader?
        $non_leading_point_a = 1;
        $non_leading_point_b = 2;
        if($leading_point == 1) {
            $non_leading_point_a = 0;
        }
        if($leading_point == 2) {
            $non_leading_point_b = 0;
        }

        //return leading point instead?
        if($return_leading_point) {
            return $leading_point;
        }

        $mid_of_non_leading_points = new stdClass();
        $mid_of_non_leading_points->x = $this->value_between_two_values($this->most_recent_object[$non_leading_point_a]->x,$this->most_recent_object[$non_leading_point_b]->x);
        $mid_of_non_leading_points->y = $this->value_between_two_values($this->most_recent_object[$non_leading_point_a]->y,$this->most_recent_object[$non_leading_point_b]->y);

        $change_in_y = $this->most_recent_object[$leading_point]->y - $mid_of_non_leading_points->y;
        $change_in_x = $this->most_recent_object[$leading_point]->x - $mid_of_non_leading_points->x;

        $radians = abs(atan($change_in_y/$change_in_x));
        if($change_in_x >= 0 && $change_in_y >= 0) {
            $this->orientation_quadrent = 1;
            //no action
        }
        elseif($change_in_x < 0 && $change_in_y >= 0) {
            $this->orientation_quadrent = 2;
            $radians = pi() - $radians;
        }
        elseif($change_in_x < 0 && $change_in_y < 0) {
            $this->orientation_quadrent = 3;
            $radians += pi();
        }
        elseif($change_in_x > 0 && $change_in_y < 0) {
            $this->orientation_quadrent = 4;
            $radians = (2 * pi()) - $radians;
        }

        $this->orientation_in_radians = $radians;
    }
    //end determine_objects_orientation

    public function ret_orientation_in_radians() {
        return $this->orientation_in_radians;
    }

    //value_between_two_values
    private function value_between_two_values($val,$valb) {
        $min = $val;
        if($valb < $min) {
            $min = $valb;
        }

        $diff = abs($val - $valb);

        return $min + ($diff / 2);
    }
    //end value_between_two_values

    //distance_between_two_points
    private function distance_between_two_points($pointa,$pointb) {
        return sqrt(pow(abs($pointa->x - $pointb->x),2) + pow(abs($pointa->y - $pointb->y),2));
    }
    //end distance_between_two_points

    //type_to_english
    private function type_to_english($type) {
        if($type == "1") return "down";
        if($type == "2") return "up";
        if($type == "3") return "move";
        return $type;
    }
//end type_to_english
}