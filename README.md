# quadMonitor
Creates a monitor object for each class containing subscribers to important ROS topics.  Some basic diagnostics (such as low SBRTK/A2D teststats or ageOfReferenceData timeouts) are performed.  A 1Hz timer also checks to see if important topics are live.  If all topics are missing, then the node prints a warning stating that the quad is "lost."

The system currently uses UDP for subscribers and does not have a TCP fallback; this is intentional.