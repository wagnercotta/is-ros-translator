def get_object_instance(key,topic,dict_or_list,type):
    if type == dict:
        return next((item for item in dict_or_list if item[key] == topic),None)
    else:
        return next((item for item in dict_or_list if getattr(item,key) == topic),None)