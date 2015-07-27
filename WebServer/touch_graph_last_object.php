<?php
/*
 * Graph's the previous object's pose on the screen.  For testing purposes.
 */

require_once $_SERVER['DOCUMENT_ROOT'] . "/db.php";
require_once $_SERVER['DOCUMENT_ROOT'] . "/touch_report.php";

$tr = new touch_report();
$last_object = $tr->most_recent_object();

if(!$last_object) {
    echo "last object is false...";
    exit;
}
?>
<html>
<head>
    <script type="text/javascript" src="https://www.google.com/jsapi"></script>
    <script type="text/javascript">
        google.load("visualization", "1.1", {packages:["corechart"]});
        google.setOnLoadCallback(drawChart);
        function drawChart() {
            var data = google.visualization.arrayToDataTable
                ([['X', 'Y'],
                    <?php
                        $first = true;
                        foreach($last_object as $point) {
                            if(!$first) {
                                echo ",\n";
                            }
                            $first = false;
                            echo "[" . $point->x . "," . $point->y . "]";
                        }

                     ?>
                ]);

            var options = {
                legend: 'none',
                hAxis: { minValue: 0, maxValue: 1 },
                vAxis: { minValue: 0, maxValue: 1 },
                curveType: 'function',
                pointSize: 10,
            };

            var chart = new google.visualization.LineChart(document.getElementById('chart_div'));
            chart.draw(data, options);
        }
    </script>
</head>
<body>
<div>Orientation: <?=round(rad2deg($tr->ret_orientation_in_radians()));?> degrees</div>
<div id="chart_div" style="width: 900px; height: 506px;"></div>
</body>
</html>