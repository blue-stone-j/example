

/* The issue is that std::atomic is neither copyable nor movable, while std::vector::resize() (when it grows) requires 
   the element type to be either copy-constructible or move-constructible so it can fill new slots. Since 
   std::atomic<bool> only has a default constructor and is not copyable/movable, resize() is not allowed.
*/
int main(int argc, char **argv)
{
  return 0;
}