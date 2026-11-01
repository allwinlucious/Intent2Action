import json

from learn import chat_server


def test_parse_tool_call_and_reasoning():
  raw = (' to=self<|message|>the centre cube is blocked<|eom|><|start|>assistant to=pick<|message|>'
         '<atem:function_calls>\n<atem:invoke name="pick">\n<atem:parameter name="object_id">obj_12</atem:parameter>\n'
         '<atem:parameter name="wait">true</atem:parameter>\n</atem:invoke>\n</atem:function_calls><|eot|>')
  message = chat_server.parse(raw)
  assert message["content"] is None
  (call,) = message["tool_calls"]
  assert call["function"]["name"] == "pick"
  assert json.loads(call["function"]["arguments"]) == {"object_id": "obj_12", "wait": True}


def test_parse_user_content():
  message = chat_server.parse(' to=user<|message|>{"rung": "skip", "reason": "blocked"}<|eot|>')
  assert message == {"role": "assistant", "content": '{"rung": "skip", "reason": "blocked"}'}


def test_template_messages_turns_string_arguments_into_dicts():
  messages = [{"role": "assistant", "content": None, "tool_calls": [
      {"id": "c1", "type": "function", "function": {"name": "pick", "arguments": '{"object_id": "obj_1"}'}}]}]
  assert chat_server.template_messages(messages)[0]["tool_calls"][0]["function"]["arguments"] == {"object_id": "obj_1"}
