# Notes for 20260730

Using bv drone
According to Yikuan's instructions, we change the IP address, and change it from drone_4 to drone_3
Spent time debugging frame transform, need to use enu_to_ned (not modalai_flip)

- drone_3_VolleyBall_20260731_001123: Manual flight. Try recording collective thrust, no individual thrust yet. Drone oscillating a bit.
- drone_3_VolleyBall_20260731_001805: Manual flight. Collective thrust and individual thrust recorded.
- drone_3_SoccerBall_20260731_002628: Manual flight with soccer ball.
- drone_3_VolleyBall_20260731_003342: Manual flight with volley ball. 
- drone_3_VolleyBall_20260731_052643: Real flight with RL policy. Pipeline works, drone oscillates due to policy, no crashes. Policy doesn;t work.