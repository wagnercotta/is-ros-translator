def get_object_instance(key,topic,dict_or_list,type):
    if type == dict:
        return next((item for item in dict_or_list if item[key] == topic),None)
    else:
        return next((item for item in dict_or_list if getattr(item,key) == topic),None)
        
# def get_class( kls ):
#     kls = kls.replace('is', 'is_msgs')
#     kls = kls.replace('camera', 'camera_pb2')
#     kls = kls.replace('common', 'common_pb2')
#     kls = kls.replace('image', 'image_pb2')
#     kls = kls.replace('power', 'power_pb2')
#     kls = kls.replace('robot', 'robot_pb2')
#     kls = kls.replace('tests', 'tests_pb2')
#     kls = kls.replace('validate', 'validate_pb2')

#     parts = kls.split('.')
#     module = ".".join(parts[:-1])
#     m = __import__( module )
#     for comp in parts[1:]:
#         m = getattr(m, comp)            
#     return m