#!/usr/bin/env python3

"""
2D Waypoint Map for CBR 2025 Maze Navigation
Shows top-down view with passage annotations and navigation sequence
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch, Circle, Rectangle
import numpy as np

# Arena and maze dimensions
ARENA_SIZE = 8.0
MAZE_X_START = 1.5  # Maze starts at x=1.5m
MAZE_X_END = 8.0  # Maze ends at x=8.0m (6.5m long)
MAZE_Y_START = 0.0
MAZE_Y_END = 2.0

# Takeoff area dimensions
TAKEOFF_X_START = 0.0  # Takeoff area starts at x=0
TAKEOFF_X_END = 1.5  # Takeoff area ends at x=1.5m
TAKEOFF_Y_START = 0.0  # Takeoff area starts at y=0
TAKEOFF_Y_END = 2.0  # Takeoff area ends at y=2m

# Waypoint positions (x, y) - adjusted for correct maze position
WAYPOINTS = [
    (2, 1.5),  # 0: Entry (first waypoint after entering)
    (2, 0.5),  # 1: After moving South
    (3, 0.5),  # 2: After moving East
    (3, 1.5),  # 3: After moving North
    (4, 1.5),  # 4: After moving East
    (4, 0.5),  # 5: After moving South
    (5, 0.5),  # 6: After moving East
    (5, 1.5),  # 7: After moving North
    (6, 1.5),  # 8: After moving East
    (6, 0.5),  # 9: After moving South
    (7, 0.5),  # 10: After moving East
    (7, 1.5),  # 11: Exit position (after moving North)
]

# Navigation requirements from mock lidar
PASSAGES = [
    {"from": 0, "to": 1, "height": "low", "direction": "South", "color": "blue"},
    {"from": 1, "to": 2, "height": "any", "direction": "East", "color": "purple"},
    {"from": 2, "to": 3, "height": "any", "direction": "North", "color": "purple"},
    {"from": 3, "to": 4, "height": "any", "direction": "East", "color": "purple"},
    {"from": 4, "to": 5, "height": "low", "direction": "South", "color": "blue"},
    {"from": 5, "to": 6, "height": "high", "direction": "East", "color": "red"},
    {"from": 6, "to": 7, "height": "low", "direction": "North", "color": "blue"},
    {"from": 7, "to": 8, "height": "high", "direction": "East", "color": "red"},
    {"from": 8, "to": 9, "height": "any", "direction": "South", "color": "purple"},
    {"from": 9, "to": 10, "height": "low", "direction": "East", "color": "blue"},
    {"from": 10, "to": 11, "height": "any", "direction": "North", "color": "purple"},
]


def create_2d_map():
    """Create 2D top-down view of the maze with waypoints."""
    fig, ax = plt.subplots(figsize=(14, 10))

    # Draw arena boundary
    arena_rect = Rectangle(
        (0, 0), ARENA_SIZE, ARENA_SIZE, fill=False, edgecolor="black", linewidth=2
    )
    ax.add_patch(arena_rect)

    # Draw maze boundary
    maze_rect = Rectangle(
        (MAZE_X_START, MAZE_Y_START),
        MAZE_X_END - MAZE_X_START,
        MAZE_Y_END - MAZE_Y_START,
        fill=False,
        edgecolor="brown",
        linewidth=3,
    )
    ax.add_patch(maze_rect)

    # Draw grid cells (0.5m x 0.5m) for entire arena
    for x in np.arange(0, ARENA_SIZE + 0.1, 0.5):
        ax.axvline(
            x,
            ymin=0,
            ymax=1,
            color="lightgray",
            linestyle=":",
            alpha=0.3,
            linewidth=0.5,
        )
    for y in np.arange(0, ARENA_SIZE + 0.1, 0.5):
        ax.axhline(
            y,
            xmin=0,
            xmax=1,
            color="lightgray",
            linestyle=":",
            alpha=0.3,
            linewidth=0.5,
        )

    # Draw 1m grid lines (stronger)
    for x in np.arange(0, ARENA_SIZE + 0.1, 1.0):
        ax.axvline(
            x,
            ymin=0,
            ymax=1,
            color="gray",
            linestyle="--",
            alpha=0.4,
            linewidth=1,
        )
    for y in np.arange(0, ARENA_SIZE + 0.1, 1.0):
        ax.axhline(
            y,
            xmin=0,
            xmax=1,
            color="gray",
            linestyle="--",
            alpha=0.4,
            linewidth=1,
        )

    # Draw takeoff area (0,0) to (1.5, 2.0) - rectangular zone
    takeoff_area = Rectangle(
        (0, 0),
        1.5,  # width (x)
        2.0,  # height (y)
        facecolor="lightgreen",
        edgecolor="green",
        linewidth=2.5,
        alpha=0.3,
    )
    ax.add_patch(takeoff_area)

    # Add text label for takeoff area
    ax.text(
        0.75,  # Center of takeoff area X
        1.0,  # Center of takeoff area Y
        "TAKEOFF AREA\n(0.4m high platform)",
        ha="center",
        va="center",
        fontsize=9,
        fontweight="bold",
        color="darkgreen",
    )

    # Draw waypoints
    for i, (x, y) in enumerate(WAYPOINTS):
        # Waypoint circle
        circle = Circle(
            (x, y), 0.15, facecolor="yellow", edgecolor="black", linewidth=2, zorder=5
        )
        ax.add_patch(circle)

        # Waypoint number
        ax.text(
            x,
            y,
            str(i),
            ha="center",
            va="center",
            fontsize=10,
            fontweight="bold",
            zorder=6,
        )

        # Add position coordinates
        ax.text(
            x,
            y - 0.25,
            f"({x:.1f},{y:.1f})",
            ha="center",
            va="center",
            fontsize=7,
            alpha=0.7,
        )

    # Draw navigation paths
    for passage in PASSAGES:
        from_wp = WAYPOINTS[passage["from"]]
        to_wp = WAYPOINTS[passage["to"]]

        # Calculate arrow position (slightly offset for visibility)
        dx = to_wp[0] - from_wp[0]
        dy = to_wp[1] - from_wp[1]

        # Draw arrow
        arrow = patches.FancyArrowPatch(
            from_wp,
            to_wp,
            connectionstyle="arc3,rad=0.1",
            arrowstyle="->,head_width=0.15,head_length=0.1",
            color=passage["color"],
            linewidth=2,
            alpha=0.7,
            zorder=3,
        )
        ax.add_patch(arrow)

        # Add height annotation at midpoint
        mid_x = (from_wp[0] + to_wp[0]) / 2
        mid_y = (from_wp[1] + to_wp[1]) / 2

        height_text = {"high": "HIGH\n1.2m", "low": "LOW\n0.4m", "any": "ANY"}[
            passage["height"]
        ]

        # Offset text to avoid overlap
        offset_y = 0.1 if passage["from"] % 2 == 0 else -0.1
        ax.text(
            mid_x,
            mid_y + offset_y,
            height_text,
            ha="center",
            va="center",
            fontsize=7,
            bbox=dict(boxstyle="round,pad=0.3", facecolor=passage["color"], alpha=0.3),
        )

    # Draw entrance and exit
    ax.annotate(
        "ENTRANCE",
        xy=(WAYPOINTS[0][0], WAYPOINTS[0][1]),
        xytext=(0.5, 3.0),
        arrowprops=dict(arrowstyle="->", color="green", lw=2),
        fontsize=10,
        fontweight="bold",
        color="green",
    )

    ax.annotate(
        "EXIT",
        xy=(WAYPOINTS[-1][0], WAYPOINTS[-1][1]),
        xytext=(7.5, 2.5),
        arrowprops=dict(arrowstyle="->", color="orange", lw=2),
        fontsize=10,
        fontweight="bold",
        color="orange",
    )

    # Draw landing area (after exiting maze)
    # From exit position (6.5, 1.5):
    # - Move 1m forward (x+1)
    # - Then move 1.5m forward and 3.5m left to arena center
    exit_x, exit_y = WAYPOINTS[-1]
    landing_x = 4.0  # Arena center X
    landing_y = 4.0  # Arena center Y

    landing_circle = Circle(
        (landing_x, landing_y),
        0.3,
        facecolor="orange",
        edgecolor="red",
        linewidth=2,
        alpha=0.7,
    )
    ax.add_patch(landing_circle)
    ax.text(
        landing_x,
        landing_y,
        "LAND\n(Arena Center)",
        ha="center",
        va="center",
        fontsize=8,
        fontweight="bold",
    )

    # Add legend
    legend_elements = [
        plt.Line2D([0], [0], color="red", linewidth=2, label="High altitude (1.2m)"),
        plt.Line2D([0], [0], color="blue", linewidth=2, label="Low altitude (0.4m)"),
        plt.Line2D([0], [0], color="purple", linewidth=2, label="Any altitude"),
    ]
    ax.legend(handles=legend_elements, loc="upper right")

    # Labels and title
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_title(
        "CBR 2025 Maze Navigation - Waypoint Map\nTop-Down View",
        fontsize=14,
        fontweight="bold",
    )
    ax.grid(True, alpha=0.2)
    ax.set_xlim(-0.5, 8.5)
    ax.set_ylim(-0.5, 8.5)
    ax.set_aspect("equal")

    # Add mission summary text
    summary_text = (
        "Mission Summary:\n"
        "1. Takeoff from area (0,0)-(1.5,2) at 0.4m height\n"
        "2. Rise to 1.2m and enter maze at WP0\n"
        "3. Visit all 12 waypoints in sequence\n"
        "4. Scan at 1.2m and 0.4m heights at each WP\n"
        "5. Exit through WP11 at high altitude\n"
        "6. Move to arena center (4,4) and land"
    )
    ax.text(
        4.0,
        6.8,
        summary_text,
        fontsize=9,
        ha="center",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="wheat", alpha=0.8),
    )

    plt.tight_layout()
    plt.savefig("/tmp/maze_waypoints_2d.png", dpi=150, bbox_inches="tight")
    plt.show()

    print("2D waypoint map saved to /tmp/maze_waypoints_2d.png")


def create_3d_scanning_visualization():
    """Create 3D visualization showing scanning positions."""
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Draw waypoints at both heights
    for i, (x, y) in enumerate(WAYPOINTS):
        # High altitude points
        ax.scatter(x, y, 1.2, c="red", s=100, marker="o", alpha=0.8)
        ax.text(x, y, 1.2, f"  {i}H", fontsize=8)

        # Low altitude points
        ax.scatter(x, y, 0.4, c="blue", s=100, marker="o", alpha=0.8)
        ax.text(x, y, 0.4, f"  {i}L", fontsize=8)

        # Vertical connection
        ax.plot([x, x], [y, y], [0.4, 1.2], "gray", linestyle="--", alpha=0.5)

        # Draw scanning cones at select waypoints
        if i in [0, 6, 11]:  # Entry, middle, exit
            theta = np.linspace(0, 2 * np.pi, 20)
            radius = 0.2

            # High altitude scan
            x_circle = x + radius * np.cos(theta)
            y_circle = y + radius * np.sin(theta)
            z_high = np.full_like(theta, 1.2)
            ax.plot(x_circle, y_circle, z_high, "r-", alpha=0.3)

            # Low altitude scan
            z_low = np.full_like(theta, 0.4)
            ax.plot(x_circle, y_circle, z_low, "b-", alpha=0.3)

    # Draw navigation paths
    for passage in PASSAGES:
        from_wp = WAYPOINTS[passage["from"]]
        to_wp = WAYPOINTS[passage["to"]]

        if passage["height"] == "high":
            z = 1.2
            color = "red"
        elif passage["height"] == "low":
            z = 0.4
            color = "blue"
        else:
            z = 1.2
            color = "purple"

        ax.plot(
            [from_wp[0], to_wp[0]],
            [from_wp[1], to_wp[1]],
            [z, z],
            color=color,
            linewidth=2,
            alpha=0.7,
        )

    # Draw maze boundary
    maze_corners = [
        (2, 0, 0),
        (8, 0, 0),
        (8, 2, 0),
        (2, 2, 0),
        (2, 0, 0),
        (2, 0, 1.5),
        (8, 0, 1.5),
        (8, 2, 1.5),
        (2, 2, 1.5),
        (2, 0, 1.5),
    ]
    maze_x = [p[0] for p in maze_corners]
    maze_y = [p[1] for p in maze_corners]
    maze_z = [p[2] for p in maze_corners]
    ax.plot(maze_x[:5], maze_y[:5], maze_z[:5], "brown", linewidth=2, alpha=0.5)
    ax.plot(maze_x[5:], maze_y[5:], maze_z[5:], "brown", linewidth=2, alpha=0.5)

    # Connect corners
    for i in range(4):
        ax.plot(
            [maze_x[i], maze_x[i + 5]],
            [maze_y[i], maze_y[i + 5]],
            [maze_z[i], maze_z[i + 5]],
            "brown",
            linewidth=1,
            alpha=0.3,
        )

    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_zlabel("Height (meters)")
    ax.set_title(
        "3D Scanning Positions and Navigation Path", fontsize=12, fontweight="bold"
    )

    ax.set_xlim(0, 8)
    ax.set_ylim(0, 8)
    ax.set_zlim(0.0, 2)

    plt.savefig("/tmp/maze_scanning_3d.png", dpi=150, bbox_inches="tight")
    plt.show()

    print("3D scanning visualization saved to /tmp/maze_scanning_3d.png")


def create_mission_summary_table():
    """Create a table summarizing the mission waypoints and actions."""
    import pandas as pd

    data = []
    for i, (x, y) in enumerate(WAYPOINTS):
        # Find passage info
        passage_to = None
        height_req = None
        direction = None

        for p in PASSAGES:
            if p["from"] == i:
                passage_to = p["to"]
                height_req = p["height"]
                direction = p["direction"]
                break

        data.append(
            {
                "Waypoint": i,
                "Position": f"({x:.1f}, {y:.1f})",
                "High Scan (1.2m)": "✓",
                "Low Scan (0.4m)": "✓",
                "Next WP": passage_to if passage_to is not None else "END",
                "Navigation Height": height_req if height_req else "-",
                "Direction": direction if direction else "-",
                "QR Codes": "Scan for A-E",
            }
        )

    df = pd.DataFrame(data)

    # Save to CSV
    df.to_csv("/tmp/maze_mission_summary.csv", index=False)
    print("Mission summary saved to /tmp/maze_mission_summary.csv")

    # Display
    print("\nMission Waypoint Summary:")
    print(df.to_string(index=False))

    return df


def main():
    """Main entry point for waypoint visualization."""
    print("Creating maze waypoint map...")
    print("-" * 50)

    # Create 2D waypoint map
    create_2d_map()

    # Create mission summary table
    create_mission_summary_table()

    print("-" * 50)
    print("Visualization created successfully!")
    print("\nFiles saved:")
    print("- /tmp/maze_waypoints_2d.png - Top-down waypoint map")
    print("- /tmp/maze_mission_summary.csv - Mission details")


if __name__ == "__main__":
    main()
