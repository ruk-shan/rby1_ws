import requests

def get_weather(city):
    """
    Retrieves the current weather for a given city using the OpenWeatherMap API.

    Args:
        city: The name of the city.

    Returns:
        A dictionary containing the weather information, or None if an error occurs.
    """
    api_key = "YOUR_API_KEY"  # Replace with your actual API key
    base_url = "http://api.openweathermap.org/data/2.5/weather?"
    complete_url = base_url + "appid=" + api_key + "&q=" + city
    response = requests.get(complete_url)
    x = response.json()
    if x["cod"] != "404":
        y = x["main"]
        current_temperature = y["temp"]
        current_pressure = y["pressure"]
        current_humidity = y["humidity"]
        z = x["weather"]
        weather_description = z[0]["description"]
        weather_data = {
            "temperature": current_temperature,
            "pressure": current_pressure,
            "humidity": current_humidity,
            "description": weather_description,
        }
        return weather_data
    else:
        return None
