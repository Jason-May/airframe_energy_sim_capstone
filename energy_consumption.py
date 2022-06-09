from cProfile import label
from turtle import color
import numpy
from matplotlib import pyplot as plt

def envelope_drag_from_vol_vel(vol, vel):
    """
    calculate the drag of an envelope with a given volume
    travelling at a given airspeed

    vol is the volume of the aircraft in ft^3
    vel is the speed of the aircraft in mph

    returns drag force in lbs
    """
    # from solidworks simulation, for an envelope with a volume of
    # 307.9 ft^3, the drag equation is Fdrag=0.013862*(vel)^2

    # from war dept manual of airship aerodynamics, Fdrag=C*(vol^2/3)*(vel^1.86)
    # the simulation data from solidworks matches better with vel^2 than vel^1.86
    # at vol=307.9 ft, Fdrag = .013862*vel^2 = C*(307.9^2/3)*(vel^1.86)
    # C*(vol^2/3) = .013862  -> C = .013862/(307.9^2/3)
    C = .013862/numpy.power(307.9,2/3)
    Fdrag = C*numpy.power(vol,2/3)*numpy.power(vel,2)
    return Fdrag

def PowerFromThrustandPropDiameter(totalThrust, propDiameter, propEfficiency=0.6):
    """
     power required to hover from the given parameters

    :param totalThrust: the total thrust required to fly in N
    :param propDiameter: the diameter of the propellor in Feet
    :param propEfficiency: the efficiency of the propellor
    :return power: the power req of the system in watts
    """
    thrust = totalThrust / 2  # to account for two propellors
    propDiameter_inMeters = propDiameter * 0.305
    power = ((2*( thrust**3)) / (3.1415*(propEfficiency**2)*(propDiameter_inMeters**2)*1.225)) ** (1/2)
    return power


# point about which other parameters are kept
# constant as one parameter is varied
############################################
center_volume = 384
center_airspeed = 16
center_net_weight_des = 3.96
############################################



def FlightCycle_Energy(helium_volume = center_volume, flight_speed = center_airspeed, net_weight_des = None):
    # parameters of payload
    payload_power_consumption = 14.425 # in watts (from power budget)

    # aircraft weight
    w_payload = 6.55 #lbs
    w_airframe = 8.42 + 4.74 #lbs (8.42 from airframe, 4.74 from non-battery electrical components)
    w_wings = 2 #lbs
    w_batteries = 3.5 #lbs
    weight_aircraft = w_payload + w_airframe + w_wings + w_batteries #lbs

    # the function can either define a helium volume from which net weight can be calculated
    # or the function can define a desired net weight from which the helium volume can be calculated
    if net_weight_des==None:
        helium_lift = 0.069*helium_volume # in lbf
        net_weight = weight_aircraft - helium_lift # in lb
    else:
        # helium_lift = weight_aircraft - net_weight_des
        # helium_volume = helium_lift/0.069
        net_weight = net_weight_des

    # for now, the wing drag is constant, but the wing will eventually be calculated to
    # be pitched to produce the needed lift
    # wing force calculation TODO, need to calculate wing variation based upon net weight
    wing_area = 750 # in sq inches
    wing_area_m_2 = wing_area*0.00064516 # converting into metric units to calculate drag/lift
    flight_speed_ms = flight_speed*0.44704

    coeff_lift = 1.390492 # lift coeff for wing
    coeff_drag = 0.158147 # drag coeff for wing
    rho_metric = 1.225 # kg/m^3
    wing_drag_n = .5*rho_metric*wing_area_m_2*coeff_drag*(flight_speed_ms**2) # wing drag in newtons
    wing_drag = wing_drag_n*0.2248 # wing drag in lbf
    wing_drag = wing_drag*2 # there are two wings


    # thrust electrical efficiency calculation
    propeller_efficiency = .43 # conversion rate of rotational power to thrust power
    electrical_efficiency = .9 # conversion rate of electrical power to rotational power
    thrust_efficiency = propeller_efficiency*electrical_efficiency # convesion from electrical power to thrust power

    # calculating flight power consumption
    envelope_drag = envelope_drag_from_vol_vel(helium_volume,flight_speed) #in lbf
    payload_drag = .2 #in lbf
                    #total estimate
    F_thrust_flight = wing_drag + envelope_drag + payload_drag #in lbf

    # print(F_thrust_flight)
    thrust_power_consumption = ((F_thrust_flight*flight_speed)/thrust_efficiency)*1.98853 # (force*velocity/efficiency)*conversion to watts factor
                                                                                # 1.98853 is the conversion from lbf*mph -> watts
    actuator_power_consumption = 8.4*2 # in watts (from power budget)
                                    # nominal value, likely not accurate in the slightest
    P_flight =  thrust_power_consumption + payload_power_consumption + actuator_power_consumption # in watts
    # print("Flight Power: ",P_flight)

    # calculating hover power consumption
    net_weight_newtons = net_weight*4.44822 # force required to hover in newtons
    if (net_weight_newtons > .01):
        hover_thrust_power_consumption = PowerFromThrustandPropDiameter(net_weight_newtons,14/12,thrust_efficiency)
    else:
        hover_thrust_power_consumption = 0
    P_hover = hover_thrust_power_consumption + payload_power_consumption # in watts
    # print("Hover Power: ",P_hover)

    # parameters of flight requirements
    hover_time = 20 # in minutes
    flight_distance = 20 # in miles
    flight_time = (flight_distance/flight_speed)*60 # in minutes

    E_flight = (P_flight*flight_time + P_hover*hover_time)/60 # energy consumed in watt hours
                                                            # energy = power*time, flight time and hover time are in minutes
                                                            # the result is divided by 60 to convert from watt minutes to watt hours

    E_payload = (hover_time + flight_time)*payload_power_consumption/60
    # E_motors_actuators = E_flight - E_payload
    E_motors_actuators = (thrust_power_consumption*flight_time + hover_thrust_power_consumption*hover_time)/60

    # print("Flight Energy: ",E_flight)
    return(E_flight,E_payload,E_motors_actuators)

FlightCycle_Energy()

hel_volume = numpy.arange(150, 450, 1)
vol_vary_energy,vol_vary_energy_payload,vol_vary_energy_motors = zip(*[FlightCycle_Energy(helium_volume=vol) for vol in hel_volume])
plt.plot(hel_volume, vol_vary_energy,label="Total Energy Consumption")
plt.plot(hel_volume, vol_vary_energy_payload,label="Payload Energy Consumption")
plt.plot(hel_volume,vol_vary_energy_motors,label="Motors + Actuators Energy Consumption")
plt.vlines(center_volume, min(vol_vary_energy+vol_vary_energy_payload+vol_vary_energy_motors),\
    max(vol_vary_energy+vol_vary_energy_payload+vol_vary_energy_motors),colors='r',label=("Current Design Volume: "+str(center_volume)+"ft^3"))
plt.xlabel("Helium Volume (ft^3)")
plt.ylabel("Energy Consumption (W-hr)")
plt.title("Energy Consumption as Helium Volume Varies")
plt.legend()

airspeed = numpy.arange(1, 20, .1)
airspeed_vary_energy,airspeed_vary_energy_payload,airspeed_vary_energy_motors = zip(*[FlightCycle_Energy(flight_speed=speed) for speed in airspeed])
plt.figure()
plt.plot(airspeed, airspeed_vary_energy,label="Total Energy Consumption")
plt.plot(airspeed, airspeed_vary_energy_payload,label="Payload Energy Consumption")
plt.plot(airspeed, airspeed_vary_energy_motors,label="Motors + Actuators Energy Consumption")
plt.vlines(center_airspeed, min(airspeed_vary_energy+airspeed_vary_energy_payload+airspeed_vary_energy_motors),\
    max(airspeed_vary_energy+airspeed_vary_energy_payload+airspeed_vary_energy_motors),colors='r',label=("Current Design Airspeed: "+str(center_airspeed)+"mph"))
plt.xlabel("Airspeed (mph)")
plt.ylabel("Energy Consumption (W-hr)")
plt.title("Energy Consumption as Airspeed Varies")
plt.legend()

netweight = numpy.arange(.1, 10, .1)
netweight_vary_energy,netweight_vary_energy_payload,netweight_vary_energy_motors = zip(*[FlightCycle_Energy(net_weight_des=weight) for weight in netweight])
plt.figure()
plt.plot(netweight, netweight_vary_energy,label="Total Energy Consumption")
plt.plot(netweight, netweight_vary_energy_payload,label="Payload Energy Consumption")
plt.plot(netweight, netweight_vary_energy_motors,label="Motors + Actuators Energy Consumption")
plt.vlines(center_net_weight_des, min(netweight_vary_energy+netweight_vary_energy_payload+netweight_vary_energy_motors),\
    max(netweight_vary_energy+netweight_vary_energy_payload+netweight_vary_energy_motors),colors='r',label=("Current Design Net Weight: "+str(center_net_weight_des)+"lbs"))
plt.xlabel("Net Weight (lbs)")
plt.ylabel("Energy Consumption (W-hr)")
plt.title("Energy Consumption as Airframe Weight Varies")
plt.legend()

plt.show()

print(FlightCycle_Energy())