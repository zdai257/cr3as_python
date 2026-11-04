# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""
Error-code module
Fetch robot alarms over HTTP
"""

import requests
import json
from typing import List, Dict, Optional, Any

# Supported languages
SUPPORTED_LANGUAGES = [
  ("Chinese", "zh_cn"),
  ("English", "en"),
  ("Japanese", "ja"),
  ("Deutsch", "de"),
  ("Español", "es"),
  ("Русский", "ru"),
  ("Korean", "ko"),
  ("Traditional Chinese", "zh_hant"),
  ("Vietnamese", "vi"),
  ("French", "fr")
]


class ErrorController:
  """
  Error controller
  
  Fetch robot alarms over HTTP  
  Example:
    error_ctrl = ErrorController(load_robot_config().ip)
    error_info = error_ctrl.GetError("zh_cn")
    formatted_error = error_ctrl.GetErrorFormatted("zh_cn")
  """
  
  def __init__(self, ip: str):
    """
    Initialize the error controller
    
    Args:
      ip: Robot IP address
    """
    self.ip = ip
  
  def SetLanguage(self, language: str = "zh_cn") -> bool:
    """
    Set robot language
    
    Args:
      language: Language code
           "zh_cn" - Simplified Chinese           "zh_hant" - Traditional Chinese 
           "en" - English
           "ja" - Japanese
           "de" - German
           "vi" - Vietnamese           "es" - Spanish
           "fr" - French
           "ko" - Korean
           "ru" - Russian
    
    Returns:
      True if the language was set
    """
    try:
      language_url = f"http://{self.ip}:22000/interface/language"
      language_data = {"type": language}
      
      response = requests.post(language_url, json=language_data, timeout=5)
      return response.status_code == 200
    except requests.exceptions.RequestException as e:
      print(f"Set language failed: {e}")
      return False
  
  def GetError(self, language: str = "zh_cn") -> Dict[str, Any]:
    """
    Get robot alarms    
    Args:
      language: Language, default "zh_cn"
    
    Returns:
      dict: Alarm dict, format:
      {
        "errMsg": [
          {
            "id": xxx,
            "level": xxx,
            "description": "xxx",
            "solution": "xxx",
            "mode": "xxx",
            "date": "xxxx",
            "time": "xxxx"
          }
        ]
      }
      If no alarm, returns{"errMsg": []}
    """
    try:
      # set language first
      self.SetLanguage(language)
      
      # getAlarminfo
      alarm_url = f"http://{self.ip}:22000/protocol/getAlarm"
      response = requests.get(alarm_url, timeout=5)
      
      if response.status_code == 200:
        try:
          result = response.json()
          # if emptyobject，normalize the payload
          if result == {} or result is None:
            return {"errMsg": []}
          return result
        except json.JSONDecodeError:
          # if emptyresponseor notJSONformat，treat as noAlarm
          return {"errMsg": []}
      else:
        print(f"getAlarminfofailed: HTTP {response.status_code}")
        return {"errMsg": []}
        
    except requests.exceptions.RequestException as e:
      print(f"HTTPrequestfault: {e}")
      return {"errMsg": []}
    except Exception as e:
      print(f"getAlarmwhile fetchingUnknownError {e}")
      return {"errMsg": []}
  
  def GetErrorFormatted(self, language: str = "zh_cn") -> str:
    """
    get formatted robotAlarminfo    
    Args:
      language: Language
    
    Returns:
      str: Formatted alarm string    """
    error_info = self.GetError(language)
    return self._format_error_messages(error_info)
  
  def _format_error_messages(self, error_info: Dict[str, Any]) -> str:
    """
    Format error text    
    Args:
      error_info: get_error() returnedAlarmmessage    
    Returns:
      str: formattedErrormessage string    """
    err_msg_list = error_info.get("errMsg", [])
    
    if not err_msg_list:
      return " no Alarminfo"
    
    messages = []
    for err in err_msg_list:
      error_id = err.get("id", "Unknown")
      level = err.get("level", "")
      description = err.get("description", "")
      solution = err.get("solution", "")
      mode = err.get("mode", "")
      date = err.get("date", "")
      time = err.get("time", "")
      
      msg = f"Error ID {error_id}"
      if level:
        msg += f"\nLevel: {level}"
      if description:
        msg += f"\nDescription: {description}"
      if solution:
        msg += f"\nSolution: {solution}"
      if mode:
        msg += f"\nMode: {mode}"
      if date and time:
        msg += f"\nTime: {date} {time}"
      
      messages.append(msg)
    
    return "\n\n".join(messages)
  
  # Backward-compatible aliases (snake_case -> PascalCase)
  set_language = SetLanguage
  get_error = GetError
  get_error_formatted = GetErrorFormatted


def parse_error_ids(error_response: Optional[str]) -> List[int]:
  """
  Parse GetErrorID responsestring（forTCPAPIGetErrorIDcommand)  
  Args:
    error_response: returned by the robotError IDresponse，format"0,{[1537,2048,2049]},GetErrorID();"
  
  Returns:
    parsedError IDlist  """
  if not error_response:
    return []
  
  # ifErrormessage rather thanError IDresponse，return []
  if isinstance(error_response, str):
    # check whetherErrormessage（ e.g. "Control Mode Is Not Tcp"）
    if not error_response.strip().startswith('0,'):
      print(f"gotErrormessage notError ID：{error_response}")
      return []
  
  try:
    # strip trailing";GetErrorID()"
    if error_response.endswith('GetErrorID();'):
      error_response = error_response[:-len('GetErrorID();')].strip()
    
    # format,{[1537,2048,2049]}
    # extract the list inside the braces    start = error_response.find('{[')
    end = error_response.find(']}')
    
    if start != -1 and end != -1:
      list_str = error_response[start+2:end]
      error_ids = [int(x.strip()) for x in list_str.split(',') if x.strip()]
      return error_ids
    else:
      # try parsing as int
      return [int(error_response.strip())]
  except ValueError as e:
    # parsefailed，return []
    print(f"parseError IDfailed：{e}")
    return []
  except Exception as e:
    print(f"parseError IDfailed：{e}")
    return []


# keep the legacy function wrappers，backward compatible
def set_language(ip: str, language: str = "zh_cn") -> bool:
  """
  Set robot language（legacy wrapper）  
  Args:
    ip: Robot IP address
    language: language code
  
  Returns:
    True if the language was set
  """
  return ErrorController(ip).SetLanguage(language)


def get_error(ip: str, language: str = "zh_cn") -> Dict[str, Any]:
  """
  Get robot alarms（legacy wrapper）
  
  Args:
    ip: Robot IP address
    language: Language
  
  Returns:
    Alarminfo dict
  """
  return ErrorController(ip).GetError(language)


def format_error_messages_from_http(error_info: Dict[str, Any]) -> str:
  """
  formatHTTPreturned by the APIErrorinfo（legacy wrapper）
  
  Args:
    error_info: get_error() returnedAlarmmessage  
  Returns:
    formattedErrormessage string  """
  err_msg_list = error_info.get("errMsg", [])
  
  if not err_msg_list:
    return " no Alarminfo"
  
  messages = []
  for err in err_msg_list:
    error_id = err.get("id", "Unknown")
    level = err.get("level", "")
    description = err.get("description", "")
    solution = err.get("solution", "")
    mode = err.get("mode", "")
    date = err.get("date", "")
    time = err.get("time", "")
    
    msg = f"Error ID {error_id}"
    if level:
      msg += f"\nLevel: {level}"
    if description:
      msg += f"\nDescription: {description}"
    if solution:
      msg += f"\nSolution: {solution}"
    if mode:
      msg += f"\nMode: {mode}"
    if date and time:
      msg += f"\nTime: {date} {time}"
    
    messages.append(msg)
  
  return "\n\n".join(messages)