/*
 * This file is part of Placeholder-2023, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Octobots <https://github.com/Octobots9084>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package frc.robot.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class ControlMap {
    public static final CommandJoystick DRIVER_LEFT = new CommandJoystick(0);
    public static final CommandJoystick DRIVER_RIGHT = new CommandJoystick(1);
    public static final CommandJoystick DRIVER_BUTTONS = new CommandJoystick(2);
    public static final CommandJoystick CO_DRIVER_LEFT = new CommandJoystick(3);
    public static final CommandJoystick CO_DRIVER_RIGHT = new CommandJoystick(4);
    public static final CommandJoystick CO_DRIVER_BUTTONS = new CommandJoystick(5);

    private ControlMap() {
    }
}
