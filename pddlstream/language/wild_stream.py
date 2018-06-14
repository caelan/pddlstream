# TODO: WildStream, FactStream

# class WildResult(object):
#     def __init__(self, stream_instance, output_objects):
#         self.stream_instance = stream_instance
#         #mapping = dict(list(zip(self.stream_instance.stream.inputs, self.stream_instance.input_objects)) +
#         #            list(zip(self.stream_instance.stream.outputs, output_objects)))
#         #self.certified = substitute_expression(self.stream_instance.stream.certified, mapping)
#     def get_certified(self):
#         #return self.certified
#     def __repr__(self):
#         #return '{}:{}->{}'.format(self.stream_instance.stream.name,
#         #                          str_from_tuple(self.stream_instance.input_objects),
#         #                          list(self.certified))
#
# def next_certified(self, **kwargs):
#     if self._generator is None:
#         self._generator = self.stream.gen_fn(*self.get_input_values())
#     new_certified = []
#     if isinstance(self._generator, FactGenerator):
#         new_certified += list(map(obj_from_value_expression, self._generator.generate(context=None)))
#         self.enumerated = self._generator.enumerated
#     else:
#         for output_objects in self.next_outputs(**kwargs):
#             mapping = dict(list(zip(self.stream.inputs, self.input_objects)) +
#                            list(zip(self.stream.outputs, output_objects)))
#             new_certified += substitute_expression(self.stream.certified, mapping)
#     return new_certified